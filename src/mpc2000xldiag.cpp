// license:BSD-3-Clause
// copyright-holders:R. Belmont, Vubeatz
// MPC2000XL prototype; hardware wiring/LCD adapted from MAME mpc2000.cpp.
// Native BIN boot from the V53 reset vector; panel MCU uses serial HLE.
// No firmware bytes embedded, no guest RAM patches, no Classic EXE handoff.
// Native boot to MAIN and selected host controls verified on macOS.
// Audio, sampling, storage and hardware timing remain unverified.

#include "emu.h"
#include "bus/ata/ataintf.h"
#include "bus/nscsi/devices.h"
#include "cpu/nec/v5x.h"
#include "imagedev/floppy.h"
#include "imagedev/snapquik.h"
#include "machine/74259.h"
#include "machine/input_merger.h"
#include "machine/mb87030.h"
#include "machine/mb89371.h"
#include "machine/timer.h"
#include "machine/upd765.h"
#include "sound/l7a1045_l6028_dsp_a.h"
#include "formats/hxchfe_dsk.h"
#include "formats/pc_dsk.h"
#include "util/hashing.h"
#include "emupal.h"
#include "output.h"
#include "screen.h"
#include "speaker.h"

#include <algorithm>
#include <array>
#include <deque>
#include <vector>

namespace {

class mpc2000xldiag_state : public driver_device
{
public:
	mpc2000xldiag_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag), m_cpu(*this, "maincpu"),
		  m_fdc(*this, "fdc"), m_floppy(*this, "fdc:0"),
		  m_sio(*this, "sio"), m_ata(*this, "ata"), m_dsp(*this, "dsp"), m_variation(*this, "VARIATION"),
		  m_shift_active(*this, "shift_active") { }
	void xl(machine_config &config);
	DECLARE_INPUT_CHANGED_MEMBER(panel_input);
	DECLARE_INPUT_CHANGED_MEMBER(shift_input);

protected:
	void machine_reset() override;

private:
	required_device<v53a_device> m_cpu;
	required_device<upd72069_device> m_fdc;
	required_device<floppy_connector> m_floppy;
	required_device<mb89371_device> m_sio;
	required_device<ata_interface_device> m_ata;
	required_device<l7a1045_sound_device> m_dsp;
	required_ioport m_variation;
	output_finder<> m_shift_active;
	std::array<u8, 256 * 64> m_pixels{};
	std::array<u16, 2> m_x{};
	std::array<u8, 2> m_y{}, m_command{};
	std::deque<u8> m_panel_bytes;
	u16 m_serial_shift = 0;
	u8 m_serial_bits = 0, m_adc_control = 0;
	bool m_loaded = false;
	bool m_shift_held = false, m_shift_latched = false, m_shift_sent = false;
	int m_variation_sent = -1;
	u32 m_lcd_writes = 0;
	int m_fdc_selected = 0, m_fdc_irq = 0, m_fdc_drq = 0, m_ata_irq = 0, m_ata_drq = 0;

	void recalc_storage();
	void storage_select(int state);
	void fdc_irq(int state) { m_fdc_irq = state; recalc_storage(); }
	void fdc_drq(int state) { m_fdc_drq = state; recalc_storage(); }
	void ata_irq(int state) { m_ata_irq = state; recalc_storage(); }
	void ata_drq(int state) { m_ata_drq = state; recalc_storage(); }

	void memory_map(address_map &map);
	void io_map(address_map &map);
	void dsp_map(address_map &map);
	void dsp_rom_map(address_map &map) { } // no optional flash expansion
	u8 dma_read(offs_t offset) { return m_cpu->space(AS_PROGRAM).read_byte(offset); }
	void dma_write(offs_t offset, u8 data) { m_cpu->space(AS_PROGRAM).write_byte(offset, data); }
	u16 dma_read16(offs_t offset) { return m_cpu->space(AS_PROGRAM).read_word(offset << 1); }
	void dma_write16(offs_t offset, u16 data) { m_cpu->space(AS_PROGRAM).write_word(offset << 1, data); }
	u8 disk_change();
	u8 lcd_status() { return 0; } // HLE: ready immediately, like upstream XL.
	template <int bank> void lcd_write(offs_t offset, u8 data);
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void palette(palette_device &palette) const;
	void panel_pair(u8 status, u8 data);
	void update_shift();
	void sync_variation();
	TIMER_DEVICE_CALLBACK_MEMBER(panel_tick);
	DECLARE_QUICKLOAD_LOAD_MEMBER(load_original);
	static void floppy_types(device_slot_interface &device);
	static void floppy_formats(format_registration &formats);
};

void mpc2000xldiag_state::memory_map(address_map &map)
{
	// XL board map from the pinned upstream driver: 4 MiB CPU RAM in
	// eight 512 KiB banks, interleaved with mirrors of the 512 KiB boot ROM.
	for (u32 bank = 0; bank < 8; ++bank)
		map(bank * 0x100000, bank * 0x100000 + 0x7ffff).ram();
	map(0x080000, 0x0fffff).rom().region("maincpu", 0).mirror(0x700000);
}

void mpc2000xldiag_state::io_map(address_map &map)
{
	// Native XL controller map; optional storage buses start empty.
	map(0x0000, 0x001f).m("spc", FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0x0020, 0x0023).m(m_fdc, FUNC(upd72069_device::map)).umask16(0x00ff);
	map(0x0060, 0x0063).rw(FUNC(mpc2000xldiag_state::lcd_status), FUNC(mpc2000xldiag_state::lcd_write<0>)).umask16(0x00ff);
	map(0x0080, 0x008f).m(m_dsp, FUNC(l7a1045_sound_device::map));
	map(0x00a0, 0x00a3).nopw(); // optional effects not modeled
	map(0x00c0, 0x00c0).lrw8(NAME([this]() { return m_adc_control; }), NAME([this](u8 data) { m_adc_control = data; }));
	map(0x00e0, 0x00ff).rw(m_ata, FUNC(ata_interface_device::cs1_r), FUNC(ata_interface_device::cs1_w));
	map(0x0100, 0x0103).rw(FUNC(mpc2000xldiag_state::lcd_status), FUNC(mpc2000xldiag_state::lcd_write<1>)).umask16(0x00ff);
	map(0x0120, 0x0121).nopw(); // expansion/control writes, not modeled
	map(0x0140, 0x015f).w("loledlatch", FUNC(hc259_device::write_a3)).umask16(0x00ff);
	map(0x0160, 0x017f).w("hiledlatch", FUNC(hc259_device::write_a3)).umask16(0x00ff);
	map(0x0180, 0x0187).m(m_sio, FUNC(mb89371_device::map<0>)).umask16(0x00ff);
	map(0x01a0, 0x01a7).m(m_sio, FUNC(mb89371_device::map<1>)).umask16(0x00ff);
	map(0x01e0, 0x01ff).rw(m_ata, FUNC(ata_interface_device::cs0_r), FUNC(ata_interface_device::cs0_w));
}

void mpc2000xldiag_state::storage_select(int state)
{
	m_fdc_selected = state;
	recalc_storage();
}

void mpc2000xldiag_state::recalc_storage()
{
	m_cpu->set_input_line(INPUT_LINE_IRQ3, m_fdc_selected ? m_fdc_irq : m_ata_irq);
	m_cpu->dreq_w<1>(m_fdc_selected ? m_fdc_drq : m_ata_drq);
}

void mpc2000xldiag_state::dsp_map(address_map &map)
{
	map(0, 0x01ffffff).ram(); // 32 MiB sample RAM, as in the XL reference
}

void mpc2000xldiag_state::machine_reset()
{
	m_shift_held = m_shift_latched = m_shift_sent = false;
	m_shift_active = 0;
	m_variation_sent = -1;
	m_pixels.fill(0);
	m_x.fill(0); m_y.fill(0); m_command.fill(0);
	m_lcd_writes = 0;
	m_panel_bytes.clear();
	m_serial_bits = 0; m_serial_shift = 0; m_adc_control = 0;
	m_fdc_selected = m_fdc_irq = m_fdc_drq = m_ata_irq = m_ata_drq = 0;
	storage_select(0);
	if (m_loaded) m_cpu->resume(SUSPEND_REASON_DISABLE);
	else m_cpu->suspend(SUSPEND_REASON_DISABLE, true);
	m_cpu->rxd_w(1);
	m_sio->write_cts<0>(CLEAR_LINE);
	m_sio->write_cts<1>(CLEAR_LINE);
	m_fdc->ready_w(CLEAR_LINE);
	logerror("XL DIAG: %s\n", m_loaded ? "reset with supplied ROM" : "waiting for user BIN, CPU suspended");
}

QUICKLOAD_LOAD_MEMBER(mpc2000xldiag_state::load_original)
{
	auto fail = [](const char *message) {
		return std::make_pair(std::make_error_condition(std::errc::invalid_argument), std::string(message));
	};
	if (m_loaded)
		return fail("Start a fresh process before selecting another MPC2KXL.BIN");
	if (image.length() != 0x80000)
		return fail("Expected a complete 512 KiB MPC2000XL BIN, including its boot ROM");
	std::vector<u8> file(0x80000);
	if (image.fread(file.data(), file.size()) != file.size())
		return fail("Short MPC2KXL.BIN read");
	// Verify even when MAME is invoked directly, bypassing the Python launcher.
	// if (util::sha1_creator::simple(file.data(), file.size()).as_string() !=
	// 	"382be688972fe3d85caeca99abff4b6c391347fb")
	// 	return fail("ROM identity mismatch: expected stock MPC2000XL 1.20; see docs/FIRMWARE.md");
	u16 sum = 0;
	for (u32 offset = 0; offset < 0x77ffe; offset += 2)
		sum += u16(file[offset] | (u16(file[offset + 1]) << 8));
	if (sum != u16(file[0x77ffe] | (u16(file[0x77fff]) << 8)))
		return fail("MPC2000XL OS checksum mismatch");
	std::copy(file.begin(), file.end(), memregion("maincpu")->base());
	m_loaded = true;
	// Reset the CPU and its integrated peripherals after populating ROM.
	// The native reset vector configures XA, page tables and OS RAM itself.
	// Do not import the Classic loader's registers, relocations or RAM handoff.
	m_cpu->reset();
	m_cpu->rxd_w(1);
	m_cpu->resume(SUSPEND_REASON_DISABLE);
	logerror("XL DIAG: stock 1.20 ROM loaded; native reset vector, panel HLE\n");
	return std::make_pair(std::error_condition(), std::string());
}

template <int bank> void mpc2000xldiag_state::lcd_write(offs_t offset, u8 data)
{
	if (!offset) { m_command[bank] = data; return; }
	switch (m_command[bank])
	{
	case 0x20:
		if (m_x[bank] <= 248 && m_y[bank] < 64)
		{
			for (int i = 0; i < 8; ++i)
				m_pixels[m_y[bank] * 256 + m_x[bank] + i] = BIT(data, 7 - i);
			++m_y[bank];
			if (++m_lcd_writes == 1)
				logerror("XL DIAG: first guest LCD data write; this is not proof of MAIN\n");
		}
		break;
	case 0x21: m_x[bank] = (u16(data) << 3) + bank * 160; break;
	case 0x22: m_y[bank] = 0; break;
	default: break; // contrast/drive setup follows the upstream diagnostic LCD model
	}
}

u32 mpc2000xldiag_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int y = cliprect.min_y; y <= cliprect.max_y; ++y)
		for (int x = cliprect.min_x; x <= cliprect.max_x; ++x)
			bitmap.pix(y, x) = m_pixels[y * 256 + x];
	return 0;
}

void mpc2000xldiag_state::palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(179, 194, 164));
	palette.set_pen_color(1, rgb_t(32, 42, 30));
}

u8 mpc2000xldiag_state::disk_change()
{
	auto *drive = m_floppy->get_device();
	return drive ? drive->dskchg_r() : 1;
}

void mpc2000xldiag_state::panel_pair(u8 status, u8 data)
{
	if (!m_loaded) return;
	if (m_panel_bytes.size() > 254) { logerror("XL DIAG: panel queue full\n"); return; }
	m_panel_bytes.push_back(status); m_panel_bytes.push_back(data & 0x7f);
}

INPUT_CHANGED_MEMBER(mpc2000xldiag_state::panel_input)
{
	if (oldval == newval) return;
	// Send the latest fader position before a pad strike in the same input frame.
	sync_variation();
	const u32 code = param;
	if (code < 0x100)
		panel_pair(newval ? 0x84 : 0x85, code);
	else if (code < 0x200)
	{
		if (newval) panel_pair(code == 0x100 ? 0x80 : 0x81, 1);
	}
	else
		panel_pair(0x90 | (code & 15), newval ? 0x76 : 0);
}

void mpc2000xldiag_state::sync_variation()
{
	if (!m_loaded || m_panel_bytes.size() > 250) return;
	// Stock XL 1.20 receiver at file 0xC68 accepts 86 vv into DS:01C9.
	// Use the native serial path, never write the guest's event queue.
	const int value = (std::min<u32>(m_variation->read(), 100) * 127 + 50) / 100;
	if (value != m_variation_sent)
	{
		panel_pair(0x86, value);
		m_variation_sent = value;
	}
}

void mpc2000xldiag_state::update_shift()
{
	const bool active = m_shift_held || m_shift_latched;
	m_shift_active = active;
	if (m_loaded && active != m_shift_sent)
	{
		panel_pair(active ? 0x84 : 0x85, 0x13);
		m_shift_sent = active;
	}
}

INPUT_CHANGED_MEMBER(mpc2000xldiag_state::shift_input)
{
	if (oldval == newval) return;
	if (!param) m_shift_held = bool(newval);
	else if (newval) m_shift_latched = !m_shift_latched;
	update_shift();
}

TIMER_DEVICE_CALLBACK_MEMBER(mpc2000xldiag_state::panel_tick)
{
	// XL 1.20 file 0x623..0x63C: BRC=8, SCU=4F, 32 MHz -> 31250 baud, 8N1.
	// Send to V53 RX; guest IRQ4 decoder/queues execute normally (no RAM injection).
	if (!m_loaded) return;
	if (!m_serial_bits)
	{
		// Coalesce continuous fader motion while retaining the final position.
		if (m_panel_bytes.empty()) sync_variation();
		if (m_panel_bytes.empty()) { m_cpu->rxd_w(1); return; }
		m_serial_shift = (u16(m_panel_bytes.front()) << 1) | 0x200;
		m_panel_bytes.pop_front(); m_serial_bits = 10;
	}
	m_cpu->rxd_w(m_serial_shift & 1);
	m_serial_shift >>= 1;
	--m_serial_bits;
}

void mpc2000xldiag_state::floppy_types(device_slot_interface &device)
{
	device.option_add("35hd", FLOPPY_35_HD);
}

void mpc2000xldiag_state::floppy_formats(format_registration &formats)
{
	formats.add(FLOPPY_HFE_FORMAT);
	formats.add(FLOPPY_PC_FORMAT);
}

void mpc2000xldiag_state::xl(machine_config &config)
{
	V53A(config, m_cpu, 32_MHz_XTAL);
	m_cpu->set_addrmap(AS_PROGRAM, &mpc2000xldiag_state::memory_map);
	m_cpu->set_addrmap(AS_IO, &mpc2000xldiag_state::io_map);
	m_cpu->out_hreq_cb().set(m_cpu, FUNC(v53a_device::hack_w));
	m_cpu->in_memr_cb().set(FUNC(mpc2000xldiag_state::dma_read));
	m_cpu->out_memw_cb().set(FUNC(mpc2000xldiag_state::dma_write));
	m_cpu->in_mem16r_cb().set(FUNC(mpc2000xldiag_state::dma_read16));
	m_cpu->out_mem16w_cb().set(FUNC(mpc2000xldiag_state::dma_write16));
	m_cpu->in_ior_cb<0>().set("spc", FUNC(mb89352_device::dma_r));
	m_cpu->out_iow_cb<0>().set("spc", FUNC(mb89352_device::dma_w));
	m_cpu->in_ior_cb<1>().set(m_fdc, FUNC(upd72069_device::dma_r));
	m_cpu->out_iow_cb<1>().set(m_fdc, FUNC(upd72069_device::dma_w));
	m_cpu->in_io16r_cb<1>().set(m_ata, FUNC(ata_interface_device::read_dma));
	m_cpu->out_io16w_cb<1>().set(m_ata, FUNC(ata_interface_device::write_dma));
	m_cpu->out_eop_cb().set("tc", FUNC(input_merger_device::in_w<0>)).invert();
	m_cpu->in_io16r_cb<3>().set(m_dsp, FUNC(l7a1045_sound_device::dma_r16_cb));
	m_cpu->out_io16w_cb<3>().set(m_dsp, FUNC(l7a1045_sound_device::dma_w16_cb));
	m_cpu->set_tclk(4'000'000); // upstream approximation: DSP-derived timer clock
	m_cpu->tout_handler<0>().set_inputline(m_cpu, INPUT_LINE_IRQ1);
	m_cpu->tout_handler<1>().set_inputline(m_cpu, INPUT_LINE_IRQ5);
	m_cpu->rxrdy_handler_cb().set("irq4", FUNC(input_merger_device::in_w<0>));
	INPUT_MERGER_ANY_HIGH(config, "irq4").output_handler().set_inputline(m_cpu, INPUT_LINE_IRQ4);

	MB89371(config, m_sio, 32_MHz_XTAL / 4);
	m_sio->rxrdy_handler<0>().set("irq4", FUNC(input_merger_device::in_w<1>));
	m_sio->rxrdy_handler<1>().set("irq4", FUNC(input_merger_device::in_w<2>));
	m_sio->txrdy_handler<0>().set_inputline(m_cpu, INPUT_LINE_IRQ7);
	m_sio->txrdy_handler<1>().set_inputline(m_cpu, INPUT_LINE_IRQ6);
	m_sio->dtr_handler<0>().set(FUNC(mpc2000xldiag_state::storage_select));
	INPUT_MERGER_ALL_HIGH(config, "tc").output_handler().set(m_fdc, FUNC(upd72069_device::tc_line_w));
	UPD72069(config, m_fdc, 32_MHz_XTAL / 2);
	m_fdc->set_ready_line_connected(false);
	m_fdc->set_ts_line_connected(false);
	m_fdc->intrq_wr_callback().set(FUNC(mpc2000xldiag_state::fdc_irq));
	m_fdc->drq_wr_callback().set(FUNC(mpc2000xldiag_state::fdc_drq));
	m_fdc->ts_rd_callback().set(FUNC(mpc2000xldiag_state::disk_change));
	FLOPPY_CONNECTOR(config, m_floppy, mpc2000xldiag_state::floppy_types, "35hd", mpc2000xldiag_state::floppy_formats).enable_sound(false);
	ATA_INTERFACE(config, m_ata).options(ata_devices, nullptr, nullptr, false);
	m_ata->irq_handler().set(FUNC(mpc2000xldiag_state::ata_irq));
	m_ata->dmarq_handler().set(FUNC(mpc2000xldiag_state::ata_drq));
	auto &lo(HC259(config, "loledlatch"));
	lo.q_out_cb<0>().set_output("led_after").invert();
	lo.q_out_cb<1>().set_output("led_rec").invert();
	lo.q_out_cb<2>().set_output("led_undo").invert();
	lo.q_out_cb<3>().set_output("led_play").invert();
	lo.q_out_cb<4>().set_output("led_overdub").invert();
	auto &hi(HC259(config, "hiledlatch"));
	hi.q_out_cb<0>().set_output("led_full").invert();
	hi.q_out_cb<1>().set_output("led_bankd").invert();
	hi.q_out_cb<2>().set_output("led_bankb").invert();
	hi.q_out_cb<3>().set_output("led_trackmute").invert();
	hi.q_out_cb<4>().set_output("led_nextseq").invert();
	hi.q_out_cb<5>().set_output("led_banka").invert();
	hi.q_out_cb<6>().set_output("led_bankc").invert();
	hi.q_out_cb<7>().set_output("led_levels").invert();

	auto &scsi(NSCSI_BUS(config, "scsi"));
	auto &spc(MB89352(config, "spc", 32_MHz_XTAL / 4));
	scsi.set_external_device(7, spc);
	spc.out_irq_callback().set_inputline(m_cpu, INPUT_LINE_IRQ2);
	spc.out_dreq_callback().set(m_cpu, FUNC(v53a_device::dreq_w<0>));

	auto &screen(SCREEN(config, "screen"));
	screen.set_lcd(); screen.set_refresh_hz(80);
	screen.set_size(248, 60); screen.set_visarea(0, 247, 0, 59);
	screen.set_screen_update(FUNC(mpc2000xldiag_state::screen_update));
	screen.set_palette("palette");
	PALETTE(config, "palette", FUNC(mpc2000xldiag_state::palette), 2);
	SPEAKER(config, "speaker", 2).front();
	L7A1045(config, m_dsp, 33.8688_MHz_XTAL);
	m_dsp->set_addrmap(AS_DATA, &mpc2000xldiag_state::dsp_map);
	m_dsp->set_addrmap(AS_IO, &mpc2000xldiag_state::dsp_rom_map);
	m_dsp->drq_handler_cb().set(m_cpu, FUNC(v53a_device::dreq_w<3>));
	m_dsp->add_route(l7a1045_sound_device::L6028_LEFT, "speaker", 1.0, 0);
	m_dsp->add_route(l7a1045_sound_device::L6028_RIGHT, "speaker", 1.0, 1);
	TIMER(config, "paneltx").configure_periodic(FUNC(mpc2000xldiag_state::panel_tick), attotime::from_hz(31250));
	QUICKLOAD(config, "quickload", "bin", attotime::from_msec(100)).set_load_callback(FUNC(mpc2000xldiag_state::load_original));
}

static INPUT_PORTS_START(mpc2000xldiag)
	// Process modifiers before digits when both become active in one frame.
	PORT_START("MODIFIERS")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("SHIFT held")
		PORT_CODE(KEYCODE_LSHIFT) PORT_CODE(KEYCODE_RSHIFT)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(mpc2000xldiag_state::shift_input), 0)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("SHIFT latch")
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(mpc2000xldiag_state::shift_input), 1)
	PORT_START("KEYS")
#define XL_KEY(mask, keycode, label, id) PORT_BIT(mask, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(keycode) PORT_NAME(label) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(mpc2000xldiag_state::panel_input), id)
	XL_KEY(0x0001, KEYCODE_F1, "F1", 1)
	XL_KEY(0x0002, KEYCODE_F2, "F2", 2)
	XL_KEY(0x0004, KEYCODE_F3, "F3", 3)
	XL_KEY(0x0008, KEYCODE_F4, "F4", 4)
	XL_KEY(0x0010, KEYCODE_F5, "F5", 5)
	XL_KEY(0x0020, KEYCODE_F6, "F6", 6)
	XL_KEY(0x0040, KEYCODE_MINUS, "Data wheel -", 0x100) PORT_CODE(KEYCODE_MINUS_PAD)
	XL_KEY(0x0080, KEYCODE_EQUALS, "Data wheel +", 0x101) PORT_CODE(KEYCODE_PLUS_PAD)
	XL_KEY(0x0100, KEYCODE_HOME, "MAIN", 0x0b) PORT_CODE(KEYCODE_M)
	XL_KEY(0x0200, KEYCODE_END, "OPEN WINDOW", 0x0c) PORT_CODE(KEYCODE_H)
	XL_KEY(0x0400, KEYCODE_LEFT, "Cursor left", 0x23)
	XL_KEY(0x0800, KEYCODE_RIGHT, "Cursor right", 0x22)
	XL_KEY(0x1000, KEYCODE_UP, "Cursor up", 0x24)
	XL_KEY(0x2000, KEYCODE_DOWN, "Cursor down", 0x21)
	XL_KEY(0x4000, KEYCODE_ENTER, "ENTER", 0x08) PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_START("NUMBERS")
	// XL 1.20 key table: file 0x108EE (DS:0A7E), decoded by file 0xC4C.
	XL_KEY(0x001, KEYCODE_0, "0", 0x14) PORT_CODE(KEYCODE_0_PAD)
	XL_KEY(0x002, KEYCODE_1, "1 / SONG", 0x0f) PORT_CODE(KEYCODE_1_PAD)
	XL_KEY(0x004, KEYCODE_2, "2 / MISC", 0x0e) PORT_CODE(KEYCODE_2_PAD)
	XL_KEY(0x008, KEYCODE_3, "3 / LOAD", 0x09) PORT_CODE(KEYCODE_3_PAD)
	XL_KEY(0x010, KEYCODE_4, "4 / SAMPLE", 0x11) PORT_CODE(KEYCODE_4_PAD)
	XL_KEY(0x020, KEYCODE_5, "5 / TRIM", 0x10) PORT_CODE(KEYCODE_5_PAD)
	XL_KEY(0x040, KEYCODE_6, "6 / PROGRAM", 0x0a) PORT_CODE(KEYCODE_6_PAD)
	XL_KEY(0x080, KEYCODE_7, "7 / MIXER", 0x12) PORT_CODE(KEYCODE_7_PAD)
	XL_KEY(0x100, KEYCODE_8, "8 / OTHER", 0x0d) PORT_CODE(KEYCODE_8_PAD)
	XL_KEY(0x200, KEYCODE_9, "9 / MIDI SYNC", 0x07) PORT_CODE(KEYCODE_9_PAD)
	PORT_START("EXTRAS")
	// XL retains Classic IDs 01..29, with 25 selecting bank A directly.
	// XL-only IDs 2A..2E select Next Seq, Track Mute and banks B/C/D.
	XL_KEY(0x00001, KEYCODE_O, "AFTER", 0x15)
	XL_KEY(0x00002, KEYCODE_B, "TAP TEMPO / NOTE REPEAT", 0x16)
	XL_KEY(0x00004, KEYCODE_DEL, "UNDO / REDO", 0x17)
	XL_KEY(0x00008, KEYCODE_BACKSLASH, "ERASE", 0x18)
	XL_KEY(0x00010, KEYCODE_COLON, "STEP <", 0x19)
	XL_KEY(0x00020, KEYCODE_QUOTE, "STEP >", 0x1a)
	XL_KEY(0x00040, KEYCODE_G, "GO TO", 0x1b)
	XL_KEY(0x00080, KEYCODE_F10, "REC", 0x1c) PORT_CODE(KEYCODE_J)
	XL_KEY(0x00100, KEYCODE_F11, "OVER DUB", 0x1d) PORT_CODE(KEYCODE_K)
	XL_KEY(0x00200, KEYCODE_BACKSPACE, "STOP", 0x1e)
	XL_KEY(0x00400, KEYCODE_SPACE, "PLAY", 0x1f)
	XL_KEY(0x00800, KEYCODE_PGUP, "BAR <<", 0x20)
	XL_KEY(0x01000, KEYCODE_F7, "BANK A", 0x25) PORT_CODE(KEYCODE_N)
	XL_KEY(0x02000, KEYCODE_F8, "FULL LEVEL", 0x26) PORT_CODE(KEYCODE_COMMA)
	XL_KEY(0x04000, KEYCODE_F9, "16 LEVELS", 0x27) PORT_CODE(KEYCODE_STOP)
	XL_KEY(0x08000, KEYCODE_PGDN, "BAR >>", 0x28)
	XL_KEY(0x10000, KEYCODE_F12, "PLAY START", 0x29) PORT_CODE(KEYCODE_P)
	XL_KEY(0x20000, KEYCODE_F13, "BANK B", 0x2c)
	XL_KEY(0x40000, KEYCODE_F14, "BANK C", 0x2d)
	XL_KEY(0x80000, KEYCODE_F15, "BANK D", 0x2e)
	XL_KEY(0x100000, KEYCODE_F16, "TRACK MUTE", 0x2b)
	XL_KEY(0x200000, KEYCODE_F17, "NEXT SEQUENCE", 0x2a)
	PORT_START("VARIATION")
	PORT_ADJUSTER(0, "NOTE VARIATION")
	PORT_START("PADS")
	XL_KEY(0x0001, KEYCODE_Z, "Pad 1", 0x200)
	XL_KEY(0x0002, KEYCODE_X, "Pad 2", 0x201)
	XL_KEY(0x0004, KEYCODE_C, "Pad 3", 0x202)
	XL_KEY(0x0008, KEYCODE_V, "Pad 4", 0x203)
	XL_KEY(0x0010, KEYCODE_A, "Pad 5", 0x204)
	XL_KEY(0x0020, KEYCODE_S, "Pad 6", 0x205)
	XL_KEY(0x0040, KEYCODE_D, "Pad 7", 0x206)
	XL_KEY(0x0080, KEYCODE_F, "Pad 8", 0x207)
	XL_KEY(0x0100, KEYCODE_Q, "Pad 9", 0x208)
	XL_KEY(0x0200, KEYCODE_W, "Pad 10", 0x209)
	XL_KEY(0x0400, KEYCODE_E, "Pad 11", 0x20a)
	XL_KEY(0x0800, KEYCODE_R, "Pad 12", 0x20b)
	XL_KEY(0x1000, KEYCODE_T, "Pad 13", 0x20c)
	XL_KEY(0x2000, KEYCODE_Y, "Pad 14", 0x20d)
	XL_KEY(0x4000, KEYCODE_U, "Pad 15", 0x20e)
	XL_KEY(0x8000, KEYCODE_I, "Pad 16", 0x20f)
#undef XL_KEY
INPUT_PORTS_END

ROM_START(mpc2000xldiag)
	ROM_REGION(0x80000, "maincpu", ROMREGION_ERASEFF) // populated only from user quickload
ROM_END

} // anonymous namespace

CONS(1999, mpc2000xldiag, 0, 0, xl, mpc2000xldiag, mpc2000xldiag_state, empty_init,
	"Akai", "MPC2000XL", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND)
