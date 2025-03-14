# Nintendo 64 emulator core (simplified)
import struct
import pickle

# Input/Controller classes
class Controller:
    def __init__(self):
        # 16-bit button state, bit mapping:
        # bit15 A, 14 B, 13 Z, 12 Start, 11 Up, 10 Down, 9 Left, 8 Right,
        # bit7,6: reserved, bit5 L, 4 R, 3 C-up, 2 C-down, 1 C-left, 0 C-right.
        self.buttons = 0   # no buttons pressed
        self.joy_x = 0     # analog stick X (-128 to 127)
        self.joy_y = 0     # analog stick Y (-128 to 127)
        self.connected = True  # assume connected

    def set_button_state(self, buttons_mask):
        self.buttons = buttons_mask & 0xFFFF

    def set_joystick(self, x, y):
        # Clamp values to -128..127
        if x < -128: x = -128
        if x > 127: x = 127
        if y < -128: y = -128
        if y > 127: y = 127
        self.joy_x = int(x)
        self.joy_y = int(y)

class InputSystem:
    def __init__(self, num_controllers=1):
        self.controllers = [Controller() for _ in range(num_controllers)]
        # Mark all controllers as connected by default
        for c in self.controllers:
            c.connected = True

# RSP (Reality Signal Processor) class
class RSP:
    def __init__(self):
        self.DMEM = bytearray(0x1000)  # 4KB data memory
        self.IMEM = bytearray(0x1000)  # 4KB instruction memory
        # RSP status flags (bit0: halt, bit1: broke, others unused here)
        self.status = 0x00000001  # Start halted (halt=1)

    def read_mem_byte(self, addr):
        # Read a byte from RSP memory (physical 0x04000000/0x04001000 base)
        if addr < 0x04001000:
            return self.DMEM[addr - 0x04000000]
        else:
            return self.IMEM[addr - 0x04001000]

    def write_mem_byte(self, addr, value):
        # Write a byte to RSP memory
        if addr < 0x04001000:
            self.DMEM[addr - 0x04000000] = value & 0xFF
        else:
            self.IMEM[addr - 0x04001000] = value & 0xFF

# RDP (Reality Display Processor) class
class RDP:
    def __init__(self, memory):
        self.memory = memory
        # RDP command registers
        self.dpc_start = 0
        self.dpc_end = 0
        self.dpc_current = 0
        self.dpc_status = 0  # 0 = idle (no busy flags)

    def process_commands(self):
        # Process RDP command list from dpc_start to dpc_end (simulate rendering)
        start = self.dpc_start & 0xFFFFFFFF
        end = self.dpc_end & 0xFFFFFFFF
        if end <= start:
            self.dpc_current = start
            # Signal DP interrupt (commands processed or none)
            self.memory.mi_intr |= 0x20  # DP (bit5) interrupt
            self.dpc_status = 0
            return
        addr = start
        while addr < end:
            # Read and skip each 64-bit command (8 bytes)
            _ = self.memory.read32(addr)
            _ = self.memory.read32(addr + 4)
            addr += 8
        self.dpc_current = end
        self.dpc_status = 0  # done, idle
        # Signal DP interrupt (display list complete)
        self.memory.mi_intr |= 0x20

# Audio system class
class Audio:
    def __init__(self, memory):
        self.memory = memory
        # Audio Interface registers (we keep minimal state)
        self.ai_dram_addr = 0
        self.ai_len = 0
        self.ai_control = 0
        self.ai_status = 0
        self.ai_dacrate = 0
        self.ai_bitrate = 0

    def dma_from_ram(self):
        # Simulate audio DMA transfer completion
        addr = self.ai_dram_addr & 0xFFFFFFFF
        length = self.ai_len & 0x3FFFF
        if length > 0:
            end = addr + length
            if end > len(self.memory.rdram):
                end = len(self.memory.rdram)
            _ = self.memory.rdram[addr:end]  # (Not used further)
        # Mark AI as not busy/full (clear busy/full bits if we consider them)
        self.ai_status &= ~0xC0000001
        # Signal AI interrupt (buffer done)
        self.memory.mi_intr |= 0x04  # AI (bit2) interrupt

# Main memory and bus class
class Memory:
    def __init__(self):
        self.RDRAM_SIZE = 8 * 1024 * 1024  # up to 8MB RDRAM
        self.rdram = bytearray(self.RDRAM_SIZE)
        self.rom = None       # cartridge ROM data (bytes)
        self.pif_rom = bytearray()  # PIF boot ROM (optional)
        self.pif_ram = bytearray(64)  # PIF RAM (64 bytes)
        # MI (MIPS Interface) registers
        self.mi_intr = 0        # interrupt flags (SP=bit0, SI=1, AI=2, VI=3, PI=4, DP=5)
        self.mi_intr_mask = 0    # interrupt mask (0 = all disabled initially)
        # Subsystem references (set in emulator init)
        self.rsp = None
        self.rdp = None
        self.audio = None
        self.input = None

    def load_rom(self, file_path):
        with open(file_path, "rb") as f:
            data = f.read()
        self.load_rom_data(data)

    def load_rom_data(self, data: bytes):
        # Ensure ROM data is in big-endian (.z64) format
        rom_data = bytearray(data)
        if len(rom_data) >= 4:
            magic = rom_data[:4]
            # Byteswapped (.v64) format
            if magic == b'\x37\x80\x40\x12':
                for i in range(0, len(rom_data), 2):
                    rom_data[i], rom_data[i+1] = rom_data[i+1], rom_data[i]
            # Little-endian (.n64) format
            elif magic == b'\x40\x12\x37\x80':
                for i in range(0, len(rom_data), 4):
                    rom_data[i], rom_data[i+3] = rom_data[i+3], rom_data[i]
                    rom_data[i+1], rom_data[i+2] = rom_data[i+2], rom_data[i+1]
        self.rom = rom_data

    # Memory read operations (8, 16, 32-bit)
    def read8(self, addr):
        a = addr & 0xFFFFFFFF
        if a >= 0x80000000:
            a &= 0x1FFFFFFF  # KSEG0/1 to physical
        if a < len(self.rdram):
            return self.rdram[a]
        if 0x03F00000 <= a < 0x04000000:
            # RDRAM interface regs (not fully implemented)
            return 0
        if 0x04000000 <= a < 0x04002000:
            return self.rsp.read_mem_byte(a)
        if 0x04040000 <= a < 0x04040020:
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if 0x04100000 <= a < 0x04300000:  # DP command/span regs
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if 0x04300000 <= a < 0x04300010:  # MI regs
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if 0x04400000 <= a < 0x04500000:  # VI regs
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if 0x04500000 <= a < 0x04600000:  # AI regs
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if 0x04600000 <= a < 0x04700000:  # PI regs
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if 0x04700000 <= a < 0x04800000:  # RI regs
            return 0
        if 0x04800000 <= a < 0x04900000:  # SI regs
            reg_val = self.read32(a & ~0x3)
            shift = (3 - (a & 0x3)) * 8
            return (reg_val >> shift) & 0xFF
        if self.rom and 0x10000000 <= a < 0x10000000 + len(self.rom):
            return self.rom[a - 0x10000000]
        if 0x1FC00000 <= a < 0x1FC007C0:
            # PIF ROM (if loaded)
            if self.pif_rom:
                offset = a - 0x1FC00000
                if offset < len(self.pif_rom):
                    return self.pif_rom[offset]
            return 0
        if 0x1FC007C0 <= a < 0x1FC00800:
            return self.pif_ram[a - 0x1FC007C0]
        return 0

    def read16(self, addr):
        high = self.read8(addr)
        low = self.read8(addr + 1)
        return ((high << 8) | low) & 0xFFFF

    def read32(self, addr):
        a = addr & 0xFFFFFFFF
        if a & 0x3:  # Unaligned: compose from bytes
            return ((self.read8(a) << 24) | (self.read8(a+1) << 16) |
                    (self.read8(a+2) << 8) | self.read8(a+3))
        if a >= 0x80000000:
            a &= 0x1FFFFFFF
        if a < len(self.rdram):
            return struct.unpack(">I", self.rdram[a:a+4])[0]
        if 0x03F00000 <= a < 0x04000000:
            return 0
        if 0x04000000 <= a < 0x04001000:
            off = a - 0x04000000
            return struct.unpack(">I", self.rsp.DMEM[off:off+4])[0]
        if 0x04001000 <= a < 0x04002000:
            off = a - 0x04001000
            return struct.unpack(">I", self.rsp.IMEM[off:off+4])[0]
        # SP registers
        if a == 0x04040010:  # SP_STATUS_REG
            return self.rsp.status
        if a == 0x04040000 or a == 0x04040004 or a == 0x04040008 or a == 0x0404000C:
            return 0  # SP_MEM_ADDR, SP_DRAM_ADDR, SP_RD_LEN, SP_WR_LEN (read not needed)
        if a == 0x04040014 or a == 0x04040018 or a == 0x0404001C:
            return 0
        # DP command registers
        if a == 0x04100000: return self.rdp.dpc_start & 0xFFFFFFFF
        if a == 0x04100004: return self.rdp.dpc_end & 0xFFFFFFFF
        if a == 0x04100008: return self.rdp.dpc_current & 0xFFFFFFFF
        if a == 0x0410000C: return self.rdp.dpc_status
        if 0x04100010 <= a < 0x04100020:
            return 0
        # DP span registers (not used)
        if 0x04200000 <= a < 0x04200010:
            return 0
        # MI registers
        if a == 0x04300000:  # MI_MODE_REG
            return 0
        if a == 0x04300004:  # MI_VERSION_REG
            return 0x02020102  # example constant
        if a == 0x04300008:  # MI_INTR_REG
            return self.mi_intr
        if a == 0x0430000C:  # MI_INTR_MASK_REG
            return self.mi_intr_mask
        # VI registers
        if a == 0x04400010:  # VI_CURRENT_REG
            self.mi_intr &= ~0x08  # clear VI interrupt on read
            return 0
        if 0x04400000 <= a < 0x04500000:
            return 0
        # AI registers
        if a == 0x04500000: return self.audio.ai_dram_addr
        if a == 0x04500004: return self.audio.ai_len
        if a == 0x04500008: return self.audio.ai_control
        if a == 0x0450000C:
            self.mi_intr &= ~0x04  # clear AI interrupt on read
            return self.audio.ai_status
        if a == 0x04500010: return self.audio.ai_dacrate
        if a == 0x04500014: return self.audio.ai_bitrate
        # PI registers
        if a == 0x04600000: return getattr(self, 'pi_dram_addr', 0)
        if a == 0x04600004: return getattr(self, 'pi_cart_addr', 0)
        if a == 0x04600010:
            # PI_STATUS: always idle (0)
            return 0
        if 0x04600000 <= a < 0x04700000:
            return 0
        # SI registers
        if a == 0x04800000: return getattr(self, 'si_dram_addr', 0)
        if a == 0x04800018:
            # SI_STATUS: bit12 = interrupt
            intr = 0x1000 if (self.mi_intr & 0x02) else 0
            return intr
        if 0x04800000 <= a < 0x04900000:
            return 0
        # Cartridge ROM direct read
        if self.rom and 0x10000000 <= a < 0x10000000 + len(self.rom):
            offset = a - 0x10000000
            return struct.unpack_from(">I", self.rom, offset)[0]
        # PIF ROM/RAM
        if 0x1FC00000 <= a < 0x1FC00000 + len(self.pif_rom):
            offset = a - 0x1FC00000
            if offset < len(self.pif_rom):
                return struct.unpack_from(">I", self.pif_rom, offset)[0]
        if 0x1FC007C0 <= a < 0x1FC00800:
            offset = a - 0x1FC007C0
            return struct.unpack_from(">I", self.pif_ram, offset)[0]
        return 0

    # Memory write operations (8, 16, 32-bit)
    def write8(self, addr, value):
        a = addr & 0xFFFFFFFF
        if a >= 0x80000000:
            a &= 0x1FFFFFFF
        if a < len(self.rdram):
            self.rdram[a] = value & 0xFF
            return
        if 0x04000000 <= a < 0x04002000:
            self.rsp.write_mem_byte(a, value)
            return
        # For others, perform a read-modify-write on the 32-bit aligned value
        reg_addr = a & ~0x3
        current = self.read32(reg_addr)
        shift = (3 - (a & 0x3)) * 8
        mask = 0xFF << shift
        new_val = (current & ~mask) | ((value & 0xFF) << shift)
        self.write32(reg_addr, new_val)

    def write16(self, addr, value):
        # Big-endian halfword
        self.write8(addr, (value >> 8) & 0xFF)
        self.write8(addr + 1, value & 0xFF)

    def write32(self, addr, value):
        a = addr & 0xFFFFFFFF
        if a & 0x3:  # Unaligned writes: break into bytes
            self.write8(a, (value >> 24) & 0xFF)
            self.write8(a+1, (value >> 16) & 0xFF)
            self.write8(a+2, (value >> 8) & 0xFF)
            self.write8(a+3, value & 0xFF)
            return
        if a >= 0x80000000:
            a &= 0x1FFFFFFF
        if a < len(self.rdram):
            self.rdram[a:a+4] = struct.pack(">I", value & 0xFFFFFFFF)
            return
        if 0x03F00000 <= a < 0x04000000:
            return  # ignore writes to RDRAM config regs
        if 0x04000000 <= a < 0x04001000:
            off = a - 0x04000000
            self.rsp.DMEM[off:off+4] = struct.pack(">I", value & 0xFFFFFFFF)
            return
        if 0x04001000 <= a < 0x04002000:
            off = a - 0x04001000
            self.rsp.IMEM[off:off+4] = struct.pack(">I", value & 0xFFFFFFFF)
            return
        # SP registers
        if a == 0x04040000:   # SP_MEM_ADDR_REG
            self.sp_mem_addr = value & 0xFFFFFFFF
            return
        if a == 0x04040004:   # SP_DRAM_ADDR_REG
            self.sp_dram_addr = value & 0xFFFFFFFF
            return
        if a == 0x04040008:   # SP_RD_LEN_REG
            length = (value & 0xFFF) + 1
            mem_off = self.sp_mem_addr & 0x1FFF
            dram_addr = self.sp_dram_addr & 0xFFFFFF
            # Perform DMA from RDRAM to SP memory
            if dram_addr + length > len(self.rdram):
                length = len(self.rdram) - dram_addr
            if mem_off < 0x1000:  # to DMEM
                end = mem_off + length
                if end > 0x1000: length = 0x1000 - mem_off
                self.rsp.DMEM[mem_off:mem_off+length] = self.rdram[dram_addr:dram_addr+length]
            else:  # to IMEM
                end = (mem_off - 0x1000) + length
                if end > 0x1000: length = 0x1000 - (mem_off - 0x1000)
                self.rsp.IMEM[mem_off-0x1000:mem_off-0x1000+length] = self.rdram[dram_addr:dram_addr+length]
            return
        if a == 0x0404000C:   # SP_WR_LEN_REG
            length = (value & 0xFFF) + 1
            mem_off = self.sp_mem_addr & 0x1FFF
            dram_addr = self.sp_dram_addr & 0xFFFFFF
            if mem_off < 0x1000:  # from DMEM
                if dram_addr + length > len(self.rdram):
                    length = len(self.rdram) - dram_addr
                if mem_off + length > 0x1000:
                    length = 0x1000 - mem_off
                self.rdram[dram_addr:dram_addr+length] = self.rsp.DMEM[mem_off:mem_off+length]
            else:  # from IMEM
                if dram_addr + length > len(self.rdram):
                    length = len(self.rdram) - dram_addr
                offset = mem_off - 0x1000
                if offset + length > 0x1000:
                    length = 0x1000 - offset
                self.rdram[dram_addr:dram_addr+length] = self.rsp.IMEM[offset:offset+length]
            return
        if a == 0x04040010:   # SP_STATUS_REG
            data = value & 0xFFFFFFFF
            # Bit0: clear halt, Bit1: set halt, Bit2: clear broke, Bit3: clear intr, Bit4: set intr
            if data & 0x01:  # clear halt (start RSP)
                self.rsp.status &= ~0x1   # running
                # Simulate RSP task execution
                self.rsp.status |= 0x1    # halt again (task done)
                self.rsp.status |= 0x2    # set broke flag
                self.mi_intr |= 0x01      # signal SP interrupt
            if data & 0x02:  # set halt
                self.rsp.status |= 0x1
            if data & 0x04:  # clear broke
                self.rsp.status &= ~0x2
            if data & 0x08:  # clear SP interrupt
                self.mi_intr &= ~0x01
            if data & 0x10:  # set SP intr (force interrupt)
                self.mi_intr |= 0x01
            return
        if 0x04040014 <= a < 0x04040020:
            return  # ignore writes to SP_DMA_FULL/BUSY/SEMA
        # DP command registers
        if a == 0x04100000:   # DPC_START_REG
            self.rdp.dpc_start = value & 0xFFFFFFFF
            return
        if a == 0x04100004:   # DPC_END_REG
            self.rdp.dpc_end = value & 0xFFFFFFFF
            self.rdp.process_commands()
            return
        if a == 0x0410000C:   # DPC_STATUS_REG
            # Acknowledge DP interrupt on write
            self.mi_intr &= ~0x20
            self.rdp.dpc_status = value & 0xFFFFFFFF
            return
        if 0x04100000 <= a < 0x04200000:
            return
        # MI registers
        if a == 0x0430000C:   # MI_INTR_MASK_REG
            mask = value & 0xFFFFFFFF
            # Interpret bits: even = clear mask, odd = set mask
            if mask & 0x1:   # SP set
                self.mi_intr_mask |= 0x01
            if mask & 0x2:   # SP clear
                self.mi_intr_mask &= ~0x01
            if mask & 0x4:   # SI set
                self.mi_intr_mask |= 0x02
            if mask & 0x8:   # SI clear
                self.mi_intr_mask &= ~0x02
            if mask & 0x10:  # AI set
                self.mi_intr_mask |= 0x04
            if mask & 0x20:  # AI clear
                self.mi_intr_mask &= ~0x04
            if mask & 0x40:  # VI set
                self.mi_intr_mask |= 0x08
            if mask & 0x80:  # VI clear
                self.mi_intr_mask &= ~0x08
            if mask & 0x100: # PI set
                self.mi_intr_mask |= 0x10
            if mask & 0x200: # PI clear
                self.mi_intr_mask &= ~0x10
            if mask & 0x400: # DP set
                self.mi_intr_mask |= 0x20
            if mask & 0x800: # DP clear
                self.mi_intr_mask &= ~0x20
            return
        if a == 0x04300000 or a == 0x04300004 or a == 0x04300008:
            # MI_MODE_REG, MI_VERSION_REG, MI_INTR_REG not writable here
            return
        # VI registers
        if a == 0x04400010:   # VI_CURRENT_REG
            self.mi_intr &= ~0x08  # clear VI interrupt
            return
        if 0x04400000 <= a < 0x04500000:
            return
        # AI registers
        if a == 0x04500000:   # AI_DRAM_ADDR_REG
            self.audio.ai_dram_addr = value & 0xFFFFFF
            return
        if a == 0x04500004:   # AI_LEN_REG
            self.audio.ai_len = value & 0x3FFFF
            self.audio.dma_from_ram()
            return
        if a == 0x04500008:   # AI_CONTROL_REG
            self.audio.ai_control = value & 0xFFFFFFFF
            return
        if a == 0x0450000C:   # AI_STATUS_REG
            self.mi_intr &= ~0x04  # clear AI interrupt
            self.audio.ai_status &= ~0xC0000000
            return
        if a == 0x04500010:
            self.audio.ai_dacrate = value & 0xFFFF
            return
        if a == 0x04500014:
            self.audio.ai_bitrate = value & 0xFFFF
            return
        # PI registers
        if a == 0x04600000:   # PI_DRAM_ADDR_REG
            self.pi_dram_addr = value & 0xFFFFFFFF
            return
        if a == 0x04600004:   # PI_CART_ADDR_REG
            self.pi_cart_addr = value & 0xFFFFFFFF
            return
        if a == 0x04600008:   # PI_RD_LEN_REG
            length = (value & 0xFFFFFF) + 1
            src = getattr(self, 'pi_cart_addr', 0) & 0xFFFFFFFF
            dest = getattr(self, 'pi_dram_addr', 0) & 0xFFFFFFFF
            if self.rom:
                if src < len(self.rom):
                    end = src + length
                    if end > len(self.rom): end = len(self.rom)
                    data = self.rom[src:end]
                    if dest + len(data) > len(self.rdram):
                        data = data[:len(self.rdram) - dest]
                    self.rdram[dest:dest+len(data)] = data
            self.mi_intr |= 0x10  # PI interrupt
            return
        if a == 0x0460000C:   # PI_WR_LEN_REG
            length = (value & 0xFFFFFF) + 1
            src = getattr(self, 'pi_dram_addr', 0) & 0xFFFFFFFF
            dest = getattr(self, 'pi_cart_addr', 0) & 0xFFFFFFFF
            if self.rom and dest < len(self.rom):
                data = self.rdram[src:src+length]
                end = dest + len(data)
                if end > len(self.rom):
                    data = data[:len(self.rom) - dest]
                self.rom[dest:dest+len(data)] = data
            self.mi_intr |= 0x10
            return
        if a == 0x04600010:   # PI_STATUS_REG
            if value & 0x02:
                self.mi_intr &= ~0x10  # clear PI interrupt
            return
        # SI registers
        if a == 0x04800000:   # SI_DRAM_ADDR_REG
            self.si_dram_addr = value & 0xFFFFFF
            return
        if a == 0x04800004:   # SI_PIF_ADDR_RD64_REG
            # DMA from PIF RAM to RDRAM (read controller data)
            dest = getattr(self, 'si_dram_addr', 0) & 0xFFFFFFFF
            length = 64
            if dest + 64 > len(self.rdram):
                length = len(self.rdram) - dest
            self.rdram[dest:dest+length] = self.pif_ram[:length]
            # Signal SI interrupt
            self.mi_intr |= 0x02
            return
        if a == 0x04800010:   # SI_PIF_ADDR_WR64_REG
            # DMA from RDRAM to PIF RAM (write controller command block)
            src = getattr(self, 'si_dram_addr', 0) & 0xFFFFFFFF
            length = 64
            if src + 64 > len(self.rdram):
                length = len(self.rdram) - src
            self.pif_ram[:length] = self.rdram[src:src+length]
            # Process controller commands in PIF RAM and prepare responses
            for i, controller in enumerate(self.input.controllers):
                if i >= 4: break
                base = 4 + 8 * i  # response bytes start at offset 4 within each 8-byte block
                if controller.connected:
                    mask = controller.buttons
                    hi = (mask >> 8) & 0xFF
                    lo = mask & 0xFF
                    joyx = controller.joy_x & 0xFF
                    joyy = controller.joy_y & 0xFF
                    self.pif_ram[base] = hi
                    self.pif_ram[base+1] = lo
                    self.pif_ram[base+2] = joyx
                    self.pif_ram[base+3] = joyy
                else:
                    # Mark as no controller: 0xFFFF for buttons, 0x00 for joystick
                    self.pif_ram[base] = 0xFF
                    self.pif_ram[base+1] = 0xFF
                    self.pif_ram[base+2] = 0x00
                    self.pif_ram[base+3] = 0x00
            # (SI interrupt will be raised when RD64 is issued)
            return
        if a == 0x04800018:   # SI_STATUS_REG
            self.mi_intr &= ~0x02  # clear SI interrupt
            return
        # PIF RAM writes
        if 0x1FC007C0 <= a < 0x1FC00800:
            offset = a - 0x1FC007C0
            # Write 4 bytes (assuming aligned)
            self.pif_ram[offset:offset+4] = struct.pack(">I", value & 0xFFFFFFFF)
            return

# VR4300 CPU class (MIPS R4300i core)
import ctypes
class VR4300:
    def __init__(self, memory):
        self.mem = memory
        self.pc = 0x80000000  # initial PC (after PIF boot code)
        self.gpr = [0] * 32     # 64-bit general registers
        self.hi = 0
        self.lo = 0
        # CP0 registers (only a few used)
        self.cp0 = [0] * 32
        # Initialize CP0 PRId (Processor ID) if needed (optional)
        self.cp0[15] = 0x00000B00  # example PRId for VR4300
        # Delay slot handling
        self.delay_slot_target = None

    def step(self):
        instr = self.mem.read32(self.pc)
        current_pc = self.pc
        next_pc = (self.pc + 4) & 0xFFFFFFFF
        in_delay_slot = (self.delay_slot_target is not None)
        target_after_slot = self.delay_slot_target
        self.delay_slot_target = None

        op = (instr >> 26) & 0x3F
        rs = (instr >> 21) & 0x1F
        rt = (instr >> 16) & 0x1F
        rd = (instr >> 11) & 0x1F
        sa = (instr >> 6) & 0x1F
        funct = instr & 0x3F
        imm = instr & 0xFFFF
        simm = imm if imm < 0x8000 else imm - 0x10000

        taken = False
        branch_target = None
        link_reg = None

        # If an instruction in a delay slot tries to branch, ignore it (not taken)
        if in_delay_slot and ((op == 0x00 and funct in (0x08, 0x09, 0x0D)) or op in (0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)):
            # Ignore this branch/jump in delay slot
            taken = False
            branch_target = None
            link_reg = None
        else:
            if op == 0x00:  # SPECIAL
                if funct == 0x00:  # SLL
                    self.gpr[rd] = (self.gpr[rt] << sa) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x02:  # SRL
                    val = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = (val >> sa) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x03:  # SRA
                    val = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                    if val & (1 << 63):
                        self.gpr[rd] = ((val >> sa) | (~0xFFFFFFFFFFFFFFFF << (64 - sa))) & 0xFFFFFFFFFFFFFFFF
                    else:
                        self.gpr[rd] = (val >> sa) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x04:  # SLLV
                    sa_var = self.gpr[rs] & 0x1F
                    self.gpr[rd] = (self.gpr[rt] << sa_var) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x06:  # SRLV
                    sa_var = self.gpr[rs] & 0x1F
                    val = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = (val >> sa_var) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x07:  # SRAV
                    sa_var = self.gpr[rs] & 0x1F
                    val = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                    if val & (1 << 63):
                        self.gpr[rd] = ((val >> sa_var) | (~0xFFFFFFFFFFFFFFFF << (64 - sa_var))) & 0xFFFFFFFFFFFFFFFF
                    else:
                        self.gpr[rd] = (val >> sa_var) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x08:  # JR
                    branch_target = self.gpr[rs] & 0xFFFFFFFFFFFFFFFC
                    taken = True
                elif funct == 0x09:  # JALR
                    branch_target = self.gpr[rs] & 0xFFFFFFFFFFFFFFFC
                    taken = True
                    link_reg = rd if rd != 0 else 31
                elif funct == 0x0C:  # SYSCALL
                    return False  # stop execution (for simplicity)
                elif funct == 0x0D:  # BREAK
                    return False  # stop execution
                elif funct == 0x10:  # MFHI
                    self.gpr[rd] = self.hi
                elif funct == 0x11:  # MTHI
                    self.hi = self.gpr[rs] & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x12:  # MFLO
                    self.gpr[rd] = self.lo
                elif funct == 0x13:  # MTLO
                    self.lo = self.gpr[rs] & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x18:  # MULT
                    a = ctypes.c_int32(self.gpr[rs] & 0xFFFFFFFF).value
                    b = ctypes.c_int32(self.gpr[rt] & 0xFFFFFFFF).value
                    result = a * b
                    self.lo = ctypes.c_int64(result).value & 0xFFFFFFFFFFFFFFFF
                    self.hi = ((ctypes.c_int64(result).value >> 32) & 0xFFFFFFFFFFFFFFFF)
                elif funct == 0x19:  # MULTU
                    a = self.gpr[rs] & 0xFFFFFFFF
                    b = self.gpr[rt] & 0xFFFFFFFF
                    result = a * b
                    self.lo = result & 0xFFFFFFFFFFFFFFFF
                    self.hi = (result >> 32) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x1A:  # DIV
                    a = ctypes.c_int64(self.gpr[rs]).value
                    b = ctypes.c_int64(self.gpr[rt]).value
                    if b != 0:
                        q = a // b
                        r = a % b
                        self.lo = ctypes.c_int64(q).value & 0xFFFFFFFFFFFFFFFF
                        self.hi = ctypes.c_int64(r).value & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x1B:  # DIVU
                    a = self.gpr[rs] & 0xFFFFFFFFFFFFFFFF
                    b = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                    if b != 0:
                        q = a // b
                        r = a % b
                        self.lo = q & 0xFFFFFFFFFFFFFFFF
                        self.hi = r & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x20 or funct == 0x21:  # ADD/ADDU
                    res = (self.gpr[rs] + self.gpr[rt]) & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = res
                elif funct == 0x22 or funct == 0x23:  # SUB/SUBU
                    res = (self.gpr[rs] - self.gpr[rt]) & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = res
                elif funct == 0x24:  # AND
                    self.gpr[rd] = self.gpr[rs] & self.gpr[rt]
                elif funct == 0x25:  # OR
                    self.gpr[rd] = self.gpr[rs] | self.gpr[rt]
                elif funct == 0x26:  # XOR
                    self.gpr[rd] = self.gpr[rs] ^ self.gpr[rt]
                elif funct == 0x27:  # NOR
                    self.gpr[rd] = (~(self.gpr[rs] | self.gpr[rt])) & 0xFFFFFFFFFFFFFFFF
                elif funct == 0x2A:  # SLT
                    s_rs = ctypes.c_int64(self.gpr[rs]).value
                    s_rt = ctypes.c_int64(self.gpr[rt]).value
                    self.gpr[rd] = 1 if s_rs < s_rt else 0
                elif funct == 0x2B:  # SLTU
                    u_rs = self.gpr[rs] & 0xFFFFFFFFFFFFFFFF
                    u_rt = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = 1 if u_rs < u_rt else 0
                elif funct == 0x2C or funct == 0x2D:  # DADD/DADDU (64-bit add)
                    res = (self.gpr[rs] + self.gpr[rt]) & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = res
                elif funct == 0x2E or funct == 0x2F:  # DSUB/DSUBU (64-bit sub)
                    res = (self.gpr[rs] - self.gpr[rt]) & 0xFFFFFFFFFFFFFFFF
                    self.gpr[rd] = res
                # Other SPECIAL functions not explicitly handled are no-ops here
            elif op == 0x01:  # REGIMM
                if rt == 0x00:  # BLTZ
                    if ctypes.c_int64(self.gpr[rs]).value < 0:
                        branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                        taken = True
                elif rt == 0x01:  # BGEZ
                    if ctypes.c_int64(self.gpr[rs]).value >= 0:
                        branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                        taken = True
                elif rt == 0x10:  # BLTZAL
                    if ctypes.c_int64(self.gpr[rs]).value < 0:
                        branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                        taken = True
                        link_reg = 31
                elif rt == 0x11:  # BGEZAL
                    if ctypes.c_int64(self.gpr[rs]).value >= 0:
                        branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                        taken = True
                        link_reg = 31
            elif op == 0x02:  # J
                branch_target = ((current_pc + 4) & 0xF0000000) | ((instr & 0x03FFFFFF) << 2)
                taken = True
            elif op == 0x03:  # JAL
                branch_target = ((current_pc + 4) & 0xF0000000) | ((instr & 0x03FFFFFF) << 2)
                taken = True
                link_reg = 31
            elif op == 0x04:  # BEQ
                if self.gpr[rs] == self.gpr[rt]:
                    branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                    taken = True
            elif op == 0x05:  # BNE
                if self.gpr[rs] != self.gpr[rt]:
                    branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                    taken = True
            elif op == 0x06:  # BLEZ
                if ctypes.c_int64(self.gpr[rs]).value <= 0:
                    branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                    taken = True
            elif op == 0x07:  # BGTZ
                if ctypes.c_int64(self.gpr[rs]).value > 0:
                    branch_target = (current_pc + 4 + (simm << 2)) & 0xFFFFFFFF
                    taken = True
            elif op == 0x08:  # ADDI
                res = ctypes.c_int64(ctypes.c_int64(self.gpr[rs]).value + simm).value & 0xFFFFFFFFFFFFFFFF
                self.gpr[rt] = res
            elif op == 0x09:  # ADDIU
                res = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                self.gpr[rt] = res
            elif op == 0x0A:  # SLTI
                self.gpr[rt] = 1 if ctypes.c_int64(self.gpr[rs]).value < simm else 0
            elif op == 0x0B:  # SLTIU
                self.gpr[rt] = 1 if (self.gpr[rs] & 0xFFFFFFFFFFFFFFFF) < (simm & 0xFFFFFFFF) else 0
            elif op == 0x0C:  # ANDI
                self.gpr[rt] = self.gpr[rs] & (imm & 0xFFFFFFFF)
            elif op == 0x0D:  # ORI
                self.gpr[rt] = (self.gpr[rs] | (imm & 0xFFFFFFFF)) & 0xFFFFFFFFFFFFFFFF
            elif op == 0x0E:  # XORI
                self.gpr[rt] = (self.gpr[rs] ^ (imm & 0xFFFFFFFF)) & 0xFFFFFFFFFFFFFFFF
            elif op == 0x0F:  # LUI
                self.gpr[rt] = (imm << 16) & 0xFFFFFFFFFFFFFFFF
            elif op == 0x10:  # COP0 (system control)
                rs_field = (instr >> 21) & 0x1F
                if rs_field == 0x00:  # MFC0
                    cop_rd = rd
                    self.gpr[rt] = self.cp0[cop_rd] & 0xFFFFFFFF
                elif rs_field == 0x04:  # MTC0
                    cop_rd = rd
                    self.cp0[cop_rd] = self.gpr[rt] & 0xFFFFFFFF
                    if cop_rd == 11:
                        # Writing Compare clears timer interrupt (CP0 Cause.IP7)
                        self.cp0[13] &= ~(1 << 15)
                elif rs_field == 0x10 and funct == 0x18:  # ERET
                    # Return from exception
                    self.pc = self.cp0[14]
                    self.cp0[12] &= ~0x2  # clear EXL (exception level) in Status
                    next_pc = self.pc
                    # No delay slot for ERET (assume immediate jump)
                    taken = False
                    branch_target = None
            elif op == 0x12:  # COP2 (RSP vector unit, not accessible by main CPU on N64)
                pass
            elif op == 0x20:  # LB
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                byte_val = self.mem.read8(addr)
                if byte_val & 0x80:
                    byte_val -= 0x100
                self.gpr[rt] = byte_val & 0xFFFFFFFFFFFFFFFF
            elif op == 0x21:  # LH
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                half_val = self.mem.read16(addr)
                if half_val & 0x8000:
                    half_val -= 0x10000
                self.gpr[rt] = half_val & 0xFFFFFFFFFFFFFFFF
            elif op == 0x23:  # LW
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                word_val = self.mem.read32(addr)
                if word_val & 0x80000000:
                    word_val -= 0x100000000
                self.gpr[rt] = word_val & 0xFFFFFFFFFFFFFFFF
            elif op == 0x24:  # LBU
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                byte_val = self.mem.read8(addr)
                self.gpr[rt] = byte_val
            elif op == 0x25:  # LHU
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                half_val = self.mem.read16(addr)
                self.gpr[rt] = half_val
            elif op == 0x26:  # LWR (unaligned)
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFF
                aligned = addr & ~0x3
                existing = self.gpr[rt] & 0xFFFFFFFFFFFFFFFF
                word_val = self.mem.read32(aligned)
                offset = addr & 3
                bytes_to_load = 4 - offset
                mask = (1 << (bytes_to_load * 8)) - 1
                val_to_load = word_val & mask
                new_val = (existing >> (bytes_to_load * 8) << (bytes_to_load * 8)) | val_to_load
                self.gpr[rt] = new_val & 0xFFFFFFFFFFFFFFFF
            elif op == 0x28:  # SB
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                self.mem.write8(addr, self.gpr[rt] & 0xFF)
            elif op == 0x29:  # SH
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                self.mem.write16(addr, self.gpr[rt] & 0xFFFF)
            elif op == 0x2B:  # SW
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFFFFFFFFFF
                self.mem.write32(addr, self.gpr[rt] & 0xFFFFFFFF)
            elif op == 0x2E:  # SWR (unaligned)
                addr = (self.gpr[rs] + simm) & 0xFFFFFFFF
                aligned = addr & ~0x3
                data_val = self.gpr[rt] & 0xFFFFFFFF
                offset = addr & 3
                bytes_to_store = offset + 1
                for i in range(bytes_to_store):
                    b = (data_val >> ((3 - i) * 8)) & 0xFF
                    self.mem.write8(aligned + i, b)
            # Other opcodes (e.g., LWC1, SDC1, etc.) not implemented (no FPU in this core)
        # Handle link register update for jumps/branches
        if link_reg is not None:
            self.gpr[link_reg] = (current_pc + 8) & 0xFFFFFFFFFFFFFFFF
        # Ensure register 0 remains 0
        self.gpr[0] = 0
        # If coming out of a delay slot, apply the pending branch target
        if target_after_slot is not None:
            self.pc = target_after_slot & 0xFFFFFFFF
        else:
            self.pc = next_pc
        # If a branch/jump is taken, set up its target to execute after the next instruction (delay slot)
        if taken and branch_target is not None:
            self.delay_slot_target = branch_target & 0xFFFFFFFF
        return True

# Main N64 emulator class
class N64Emulator:
    def __init__(self):
        self.memory = Memory()
        self.rsp = RSP()
        self.input = InputSystem(num_controllers=4)
        self.audio = Audio(self.memory)
        self.rdp = RDP(self.memory)
        self.cpu = VR4300(self.memory)
        # Connect subsystems to memory
        self.memory.rsp = self.rsp
        self.memory.rdp = self.rdp
        self.memory.audio = self.audio
        self.memory.input = self.input

    def load_rom(self, rom_path):
        self.memory.load_rom(rom_path)
        # Set start PC from ROM header (offset 0x8 holds boot address for some ROMs)
        if self.memory.rom and len(self.memory.rom) >= 0xB0:
            entry_point = struct.unpack_from(">I", self.memory.rom, 0x8)[0]
            if entry_point != 0:
                self.cpu.pc = entry_point

    def save_state(self, file_path):
        state = {
            'pc': self.cpu.pc,
            'gpr': self.cpu.gpr.copy(),
            'hi': self.cpu.hi,
            'lo': self.cpu.lo,
            'cp0': self.cpu.cp0.copy(),
            'rdram': bytes(self.memory.rdram),
            'rsp_dmem': bytes(self.rsp.DMEM),
            'rsp_imem': bytes(self.rsp.IMEM),
            'rsp_status': self.rsp.status,
            'rdp': {
                'dpc_start': self.rdp.dpc_start,
                'dpc_end': self.rdp.dpc_end,
                'dpc_current': self.rdp.dpc_current,
                'dpc_status': self.rdp.dpc_status
            },
            'audio': {
                'ai_dram_addr': self.audio.ai_dram_addr,
                'ai_len': self.audio.ai_len,
                'ai_control': self.audio.ai_control,
                'ai_status': self.audio.ai_status,
                'ai_dacrate': self.audio.ai_dacrate,
                'ai_bitrate': self.audio.ai_bitrate
            },
            'mi_intr': self.memory.mi_intr,
            'mi_mask': self.memory.mi_intr_mask,
            'pi_dram_addr': getattr(self.memory, 'pi_dram_addr', 0),
            'pi_cart_addr': getattr(self.memory, 'pi_cart_addr', 0),
            'si_dram_addr': getattr(self.memory, 'si_dram_addr', 0),
            'pif_ram': bytes(self.memory.pif_ram)
        }
        with open(file_path, "wb") as f:
            pickle.dump(state, f)

    def load_state(self, file_path):
        with open(file_path, "rb") as f:
            state = pickle.load(f)
        self.cpu.pc = state['pc']
        self.cpu.gpr = state['gpr'].copy()
        self.cpu.hi = state['hi']
        self.cpu.lo = state['lo']
        self.cpu.cp0 = state['cp0'].copy()
        self.memory.rdram[:] = state['rdram']
        self.rsp.DMEM[:] = state['rsp_dmem']
        self.rsp.IMEM[:] = state['rsp_imem']
        self.rsp.status = state.get('rsp_status', self.rsp.status)
        rdp_state = state.get('rdp', {})
        self.rdp.dpc_start = rdp_state.get('dpc_start', 0)
        self.rdp.dpc_end = rdp_state.get('dpc_end', 0)
        self.rdp.dpc_current = rdp_state.get('dpc_current', 0)
        self.rdp.dpc_status = rdp_state.get('dpc_status', 0)
        audio_state = state.get('audio', {})
        self.audio.ai_dram_addr = audio_state.get('ai_dram_addr', 0)
        self.audio.ai_len = audio_state.get('ai_len', 0)
        self.audio.ai_control = audio_state.get('ai_control', 0)
        self.audio.ai_status = audio_state.get('ai_status', 0)
        self.audio.ai_dacrate = audio_state.get('ai_dacrate', 0)
        self.audio.ai_bitrate = audio_state.get('ai_bitrate', 0)
        self.memory.mi_intr = state.get('mi_intr', 0)
        self.memory.mi_intr_mask = state.get('mi_mask', 0)
        self.memory.pi_dram_addr = state.get('pi_dram_addr', 0)
        self.memory.pi_cart_addr = state.get('pi_cart_addr', 0)
        self.memory.si_dram_addr = state.get('si_dram_addr', 0)
        self.memory.pif_ram[:] = state.get('pif_ram', self.memory.pif_ram)

    def apply_cheats(self, cheats):
        # cheats can be dict {addr: value} or list of (addr, value) or (addr, value, size)
        if isinstance(cheats, dict):
            cheats_list = [(addr, val) for addr, val in cheats.items()]
        else:
            cheats_list = cheats
        for cheat in cheats_list:
            if isinstance(cheat, tuple):
                if len(cheat) == 3:
                    addr, value, size = cheat
                elif len(cheat) == 2:
                    addr, value = cheat
                    # determine write size by value
                    if value <= 0xFF:
                        size = 8
                    elif value <= 0xFFFF:
                        size = 16
                    elif value <= 0xFFFFFFFF:
                        size = 32
                    else:
                        size = 32
                else:
                    continue
                if size == 8:
                    self.memory.write8(addr, value)
                elif size == 16:
                    self.memory.write16(addr, value)
                elif size == 32:
                    self.memory.write32(addr, value)

    def run(self, max_instructions=None):
        executed = 0
        vi_counter = 0
        try:
            while True:
                if max_instructions is not None and executed >= max_instructions:
                    break
                executed += 1
                running = self.cpu.step()
                if not running:
                    break  # stop if CPU signaled break/stop
                # Very simple timing: generate VI interrupt periodically
                vi_counter += 1
                if vi_counter >= 500000:  # arbitrary threshold for one frame
                    vi_counter = 0
                    self.memory.mi_intr |= 0x08  # VI (bit3) interrupt
                # Increment CP0 Count register each step (approx)
                self.cpu.cp0[9] = (self.cpu.cp0[9] + 1) & 0xFFFFFFFF
                # If Count equals Compare, set CP0 Cause.IP7 (timer interrupt pending)
                if self.cpu.cp0[11] != 0 and self.cpu.cp0[9] == self.cpu.cp0[11]:
                    self.cpu.cp0[13] |= (1 << 15)
        except KeyboardInterrupt:
            print("Execution interrupted by user.")
        return executed

# Run the emulator if executed as a script
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python program.py [rom_file]")
        sys.exit(1)
    rom_path = sys.argv[1]
    emulator = N64Emulator()
    emulator.load_rom(rom_path)
    print(f"Running ROM: {rom_path}")
    emulator.run()
    print("Emulation ended.")
