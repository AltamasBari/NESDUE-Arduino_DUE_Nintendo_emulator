
///#include <assert.h>


//################################################################################
//################################################################################
//################################################################################
//################################################################################
//################################################################################
//################################################################################
//################################################################################
//#include "SPI.h"
#include "ILI9341_due_config.h"
#include "ILI9341_due.h"

#define LCD_CS 8  // Chip Select for LCD
#define LCD_RST 9 // Reset for LCD
#define LCD_DC 10  // Command/Data for LCD

#include "Arial_bold_14.h"

ILI9341_due tft(LCD_CS, LCD_DC, LCD_RST);
//--------------------------------------------------------------------------------
// Color set
#define BLACK           0x0000
#define RED             0xF800
#define GREEN           0x07E0
#define BLUE            0x102E
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0 
#define ORANGE          0xFD20
#define GREENYELLOW     0xAFE5 
#define DARKGREEN       0x03E0
#define WHITE           0xFFFF
//--------------------------------------------------------------------------------
//********************************************************************************
//#include <SD.h>
#include "SdFat.h"
SdFat SD;

#define SD_CS_PIN 7
File romfile;
//********************************************************************************





//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//typedefs.h
typedef unsigned char u8;
typedef char i8;

typedef unsigned short u16;
typedef short i16;

typedef unsigned int u32;
typedef int i32;
typedef unsigned int b32;

typedef unsigned long long u64;
typedef long long i64;

typedef float r32;
typedef double r64;

#define TRUE    1
#define FALSE   0 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u8 memory[64*1024]; //64kB last 32kB cartridge_PRGROM

u8 ppu_cartridge_CHR[8192]; //8kB

u8 ppu_internal_ram[2048];  //2kB
u8 ppu_spr_ram[256];
u8 ppu_palette[32];




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//nes.h
#define TV_SYSTEM_NTSC  0
#define TV_SYSTEM_PAL   1

#define SAMPLES_PER_SECOND  44100

#define NAMETABLE_MIRRORING_VERTICAL        1
#define NAMETABLE_MIRRORING_HORIZONTAL      2
#define NAMETABLE_MIRRORING_SINGLE_LOW      3
#define NAMETABLE_MIRRORING_SINGLE_HIGH     4

///typedef u8   (*cartridgeReadFunc)(u16);
///typedef void (*cartridgeWriteFunc)(u16, u8);

typedef struct Emulator
{
    u32 tvSystem;
    r64 secondsSinceStart;
    u32 ntMirroring;
    
///    u8 **prgRamBlocks;
    u32 prgRamBlockCount;
    u8* currentPrg1Ptr;
    u8* currentPrg2Ptr;

///    u8 **chrRomBlocks;
    u32 chrRomBlockCount;
    u8* currentChrLowPtr;
    u8* currentChrHiPtr;

    u8* currentNtPtr[4];
///
/*    cartridgeReadFunc crRead;
    cartridgeWriteFunc crWrite;*/
} Emulator;

typedef struct CPU
{
    // computational state
    u8 A; // accumulator
    u8 X; // index 1
    u8 Y; // index 2
    u16 PC; // program counter
    u8 S; // stack pointer
    //u8 P; // status register

    u8 carryFlag;
    u8 zeroFlag;
    u8 intFlag;
    u8 decimalFlag;
    u8 overflowFlag;
    u8 signFlag;

    // emulation state
    u8 strobe;
    u8 pad1Data;
    u8 pad2Data;

    u32 clockHz;
    u32 suspClocks; // clocks left to complete instruction

} __attribute__((packed)) CPU;

typedef struct Eval_Sprite
{
    u8 yPos;
    u8 idx;
    u8 attr;
    u8 xPos;
    u8 isSprite0;
    u8 isFirstPart; // false if second part of 8x16 sprite
} Eval_Sprite;

typedef struct Scanline_Data
{
    u32 palIdx;
    u8 indexBits;
    u8 lowSR;
    u8 highSR;
    Eval_Sprite sprites[8];
    u32 spriteCount;
} Scanline_Data;

typedef struct PPU
{
    u8 CTRL;
    u8 MASK;
    u8 STATUS;
    u8 OAM_ADDR;

    u8 OAM_DATA;
    u8 SCROLL;
    u8 ADDR;
    u8 DATA;

    u8 OAM_DMA; // last register
    u8 latchToggle;
    u8 scrollX;
    u8 scrollY;

    u8 bufferedData;
    u8 busLatch;
    u8 align1[2];

    u16 accessAdr;
    u16 nameTblAdr;
    u16 spriteTblAdr;
    u16 bgTblAdr;

    i32 scanline;
    u32 clk;

    Scanline_Data slData;

} __attribute__((packed)) PPU;

typedef struct SweepUnit
{
    u8 reload;
    u8 divider;
} SweepUnit;

typedef struct EnvelopeUnit
{
    u8 startFlag;
    u8 divider;
    u8 decay;
} EnvelopeUnit;

typedef struct APU
{
    u8 PULSE1[4];       // 0x4000
    u8 PULSE2[4];       // 0x4004
    u8 TRIANGLE[4];     // 0x4008
    u8 NOISE[4];        // 0x400C
    u8 DMC_CHANNEL_0;   // 0x4010
    u8 DMC_CHANNEL_1;
    u8 DMC_CHANNEL_2;
    u8 DMC_CHANNEL_3;   // 0x4013
    u8 NA1;
    u8 CONTROL;         // 0x4015
    u8 NA2;
    u8 FRAME_COUNTER;   // 0x4017

    // emulation variables
    u8 p1Length;
    u8 p2Length;

    EnvelopeUnit p1Env;
    EnvelopeUnit p2Env;
    EnvelopeUnit nEnv;

    SweepUnit p1Sweep;
    SweepUnit p2Sweep;

    // triangle channel
    u8 tLength;
    u8 tLinCounter;
    u8 tLinReloadFlag;
    u8 tLinCtrlFlag;

    u8 nLength;
    u16 nShift;
    u16 nState;

    u32 clocksPerFrame;
    // keeps track of how many apu clocks until next frame counter tick
    u32 frameClkCount;
    // keeps track of the 4 or 5 frame counter tick sequence
    u32 frameCounter;
} __attribute__((packed)) APU; 



APU apu;
CPU cpu;
PPU ppu;
Emulator emu;

int iddd = 0;
u8 buttons = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ppu.c

/*
u32 palette[64] =
{
    0x7C7C7CFF, 0x0000FCFF, 0x0000BCFF, 0x4428BCFF,
    0x940084FF, 0xA80020FF, 0xA81000FF, 0x881400FF,
    0x503000FF, 0x007800FF, 0x006800FF, 0x005800FF,
    0x004058FF, 0x000000FF, 0x000000FF, 0x000000FF,
    0xBCBCBCFF, 0x0078F8FF, 0x0058F8FF, 0x6844FCFF,
    0xD800CCFF, 0xE40058FF, 0xF83800FF, 0xE45C10FF,
    0xAC7C00FF, 0x00B800FF, 0x00A800FF, 0x00A844FF,
    0x008888FF, 0x000000FF, 0x000000FF, 0x000000FF,
    0xF8F8F8FF, 0x3CBCFCFF, 0x6888FCFF, 0x9878F8FF,
    0xF878F8FF, 0xF85898FF, 0xF87858FF, 0xFCA044FF,
    0xF8B800FF, 0xB8F818FF, 0x58D854FF, 0x58F898FF,
    0x00E8D8FF, 0x787878FF, 0x000000FF, 0x000000FF,
    0xFCFCFCFF, 0xA4E4FCFF, 0xB8B8F8FF, 0xD8B8F8FF,
    0xF8B8F8FF, 0xF8A4C0FF, 0xF0D0B0FF, 0xFCE0A8FF,
    0xF8D878FF, 0xD8F878FF, 0xB8F8B8FF, 0xB8F8D8FF,
    0x00FCFCFF, 0xF8D8F8FF, 0x000000FF, 0x000000FF
};
*/
u16 palette[64] = {
0x7BEF,0x001F,0x0017,0x4157,0x9010,0xA804,0xA880,0x88A0,
0x5180,0x03C0,0x0340,0x02C0,0x020B,0x0000,0x0000,0x0000,
0xBDF7,0x03DF,0x02DF,0x6A3F,0xD819,0xE00B,0xF9C0,0xE2E2,
0xABE0,0x05C0,0x0540,0x0548,0x0451,0x0000,0x0000,0x0000,
0xFFDF,0x3DFF,0x6C5F,0x9BDF,0xFBDF,0xFAD3,0xFBCB,0xFD08,
0xFDC0,0xBFC3,0x5ECA,0x5FD3,0x075B,0x7BCF,0x0000,0x0000,
0xFFFF,0xA73F,0xBDDF,0xDDDF,0xFDDF,0xFD38,0xF696,0xFF15,
0xFECF,0xDFCF,0xBFD7,0xBFDB,0x07FF,0xFEDF,0x0000,0x0000,
};

u8 ppu_get_register(u8 reg)
{
    u8 ret;
    ///assert(reg <= 7);
    switch(reg)
    { 
        case 2: // PPUSTATUS
            ret = ppu.STATUS;
            ppu.STATUS = ret & ~0x80;
            ppu.latchToggle = 0;
            break;
        case 4:
            // see description in read_oam_data()
            // unimplemented OAM bits are always 0
            if(ppu.scanline < 241)
                ret = ppu.OAM_DATA&0xE3;
            else
                ret = ppu_spr_ram[ppu.OAM_ADDR];
            break;
        case 7:
            ret = ppu_read_byte(ppu.accessAdr);
            if(ppu.accessAdr < 0x3F00)
            {
                u8 buf = ppu.bufferedData;
                ppu.bufferedData = ret;
                ret = buf;
            }
            else // the buffer will not be affected by palette ram, but read value will
                ppu.bufferedData = ppu_read_byte(ppu.accessAdr-0x1000);
            if(!(ppu.CTRL & 0x04))
                ppu.accessAdr += 1;
            else
                ppu.accessAdr += 32;
            break;
        default:
            ret = ppu.busLatch;
            break;
    }
    ppu.busLatch = ret;
    return ret;
}

void ppu_set_register(u8 reg, u8 value)
{
    //printf("ppu set register %d to %d\n", reg, value);
    ///assert(reg <= 7);
    switch(reg)
    {
        case 0:
            // TODO: remove hack (makes SMB work)
            // TODO: 8x16 sprites
            //if((ppu.MASK & 0x18) != 0)
                ppu.nameTblAdr = 0x2000 + (value & 0x3) * 0x400;
            // bit 3 = sprite table adr (8x8, ignored for 8x16) 0 = 0, 1=0x1000
            if((value & 0x08) != 0)
                ppu.spriteTblAdr = 0x1000;
            else
                ppu.spriteTblAdr = 0x0000;
            if((value & 0x10) != 0)
                ppu.bgTblAdr = 0x1000;
            else
                ppu.bgTblAdr = 0x0000;
            ///assert((value&0x40)==0);  
            ppu.CTRL = value;
            break;
        case 1:
            ppu.MASK = value;
            break;
        case 3: // OAM adr TODO: 2A03/7 support
            ppu.OAM_ADDR = value;    
            break;
        case 4: // OAM write
            // TODO: support weird unofficial behaviour
            if(ppu.scanline >= 241) // don't allow writes during rendering
            {
                ppu_spr_ram[ppu.OAM_ADDR] = value;
                ppu.OAM_ADDR++;
            }
            break;
        case 5:
            if(!ppu.latchToggle)
                ppu.scrollX = value;
            else
            {
                ///assert(value < 240); // TODO: not entirely correct
                ppu.scrollY = value;
            }
            ppu.latchToggle = !ppu.latchToggle;
            break;
        case 6:
        {
            if(!ppu.latchToggle)
                ppu.accessAdr = (value<<8)&0xFF00;
            else
                ppu.accessAdr |= value;
            ppu.latchToggle = !ppu.latchToggle;
        } break;
        case 7:
        {
            // incremend of ppu.accessAdr depens on bit 2 (3rd bit) of ppu CTRL
            // 0 - add 1 going across, 1 - add 32, going down
            u16 adrBefore = ppu.accessAdr;
            if(!(ppu.CTRL & 0x04))
                ppu.accessAdr += 1;
            else
                ppu.accessAdr += 32;
            ppu_write_byte(adrBefore, value);
        } break;
        default:
            *((u8*)&ppu + reg) = value;
            break;
    }
    ppu.busLatch = value;
}

u8 ppu_read_oam_data(u8 adr)
{
    u8 val = ppu_spr_ram[adr];
    // TODO: should also apply to scanline sprite RAM
    ppu.OAM_DATA = val; // reading OAM_DATA during render exposes internal OAM access
    return val;
}

static inline u8 ppu_read_nametable_byte(u16 adr)
{
    return emu.currentNtPtr[(adr&0xFFF)>>10][adr&0x3FF];
}

static inline void ppu_write_nametable_byte(u16 adr, u8 val)
{
    emu.currentNtPtr[(adr&0xFFF)>>10][adr&0x3FF] = val;
}

u8 ppu_read_byte(u16 adr)
{

    if(adr >= 0x0000 && adr < 0x1000)
    {
        //NCX:
        ///return emu.currentChrLowPtr[adr];
        return ppu_cartridge_CHR[adr];
    }
    else if(adr >= 0x1000 && adr < 0x2000)
    {
        ///return emu.currentChrHiPtr[adr-0x1000];
        return ppu_cartridge_CHR[adr];        
    }
    else if(adr >= 0x2000 && adr < 0x3000)
    {
        return ppu_read_nametable_byte(adr);
    }
    else if(adr >= 0x3000 && adr < 0x3F00) // mirror of 0x2000+
    {
        return ppu_read_nametable_byte(adr-0x1000);
    }
    else if(adr >= 0x3F00 && adr < 0x3FFF) // palette RAM
    {
        u32 index = adr&0x1F;
        switch(index)
        {
            case 0x10: index = 0x0; break;
            case 0x14: index = 0x4; break;
            case 0x18: index = 0x8; break;
            case 0x1C: index = 0xC; break;
            default: break;
        }
        return ppu_palette[index];
    }
    else
    {
        ///assert(FALSE); // ppu memory region not set
        return 0;
    }
}

void ppu_write_byte(u16 adr, u8 value)
{
    if(adr >= 0x0000 && adr < 0x1000) // TODO: should be read only?
    {
        ///emu.currentChrLowPtr[adr] = value;
        ///printf("ppu write RAM\n");
        ppu_cartridge_CHR[adr] = value;
    }
    else if(adr >= 0x1000 && adr < 0x2000)
    {
        ///printf("ppu write RAM 2\n");
        ///emu.currentChrHiPtr[adr-0x1000] = value;
        ppu_cartridge_CHR[adr] = value;
    }
    else if(adr >= 0x2000 && adr < 0x3000)
    {
        ppu_write_nametable_byte(adr, value);
    }
    else if(adr >= 0x3000 && adr < 0x3F00) // mirror of 0x2000+
    {
        ppu_write_nametable_byte(adr-0x1000, value);
    }
    else if(adr >= 0x3F00 && adr < 0x3FFF) // palette RAM
    {
        u32 index = adr&0x1F;
        switch(index)
        {
            case 0x10: index = 0x0; break;
            case 0x14: index = 0x4; break;
            case 0x18: index = 0x8; break;
            case 0x1C: index = 0xC; break;
            default: break;
        }
        ppu_palette[index] = value;
    }
}

static inline void ppu_eval_sprites(i32 sl)
{
    Scanline_Data *sld = &ppu.slData;
    u32 sc = 0;
    for(int i = 0; i < 64; i++)
    {
        // TODO: implement 8x16 sprites, also below when rendering
        //assert((ppu.CTRL & 0x20) == 0);
        u32 spriteY = ppu_read_oam_data(4*i);
        if(sl >= spriteY && sl < spriteY+8)
        {
            if(sc == 8) // set sprite overflow bit
            {
                ppu.STATUS |= 0x20;
                break;
            }
            ///assert(sc >= 0 && sc < 8);
            sld->sprites[sc].yPos = spriteY;
            sld->sprites[sc].idx = ppu_read_oam_data(4*i+1);
            sld->sprites[sc].attr = ppu_read_oam_data(4*i+2);
            sld->sprites[sc].xPos = ppu_read_oam_data(4*i+3);
            sld->sprites[sc].isSprite0 = (i == 0);
            sc++;
        }
    }
    sld->spriteCount = sc;
}


uint16_t ILIbuffer[4096];


static inline void ppu_render_pixel(i32 sl, i32 col)
{
    Scanline_Data *sld = &ppu.slData;
    u16 bgColor = palette[ppu_read_byte(0x3F00)];
    u8 colB = 0;

    i32 x = col;
    i32 y = sl;

    u32 ocol = col;
    col += ppu.scrollX;
    u32 nt = (col >> 8) ? ppu.nameTblAdr + 0x400 : ppu.nameTblAdr;
    col &= 0xFF;

    u32 isLeftEdge = (col < 8);

    // background
    if((ppu.MASK & 0x08) != 0 && (!isLeftEdge || (ppu.MASK&0x2) != 0))
    {
        if((col & 0x7) == 0 || col == ppu.scrollX) // get nametable data
        {
            sld->indexBits = 0;
            u32 palX = col>>5;
            u32 palY = sl>>5;
            sld->palIdx = (palY<<3)+palX;
            u8 palB = ppu_read_byte(0x03C0 + nt + sld->palIdx);
            u32 pxM = col&0x1F;
            u32 slM = sl&0x1F;
            if(pxM<16 && slM<16) // top left
                sld->indexBits |= (palB&0x3)<<2;
            else if(pxM>=16 && slM<16) // top right
                sld->indexBits |= ((palB>>2)&0x3)<<2;
            else if(pxM<16 && slM>=16) // bottom left
                sld->indexBits |= ((palB>>4)&0x3)<<2; 
            else if(pxM>=16 && slM>=16) // bottom right
                sld->indexBits |= ((palB>>6)&0x3)<<2;
            u32 ntb = ppu_read_byte(nt + (col>>3) + ((sl&(~7))<<2));

            sld->lowSR = ppu_read_byte((ntb<<4)+(sl&0x7)+ppu.bgTblAdr);
            sld->highSR = ppu_read_byte((ntb<<4)+(sl&0x7)+ppu.bgTblAdr+8);
        }
        colB = ((sld->lowSR >> (7-(col&0x7))) & 1);
        colB |= ((sld->highSR >> (7-(col&0x7))) & 1) << 1;
        if((colB & 0x3) != 0)
            bgColor = palette[ppu_read_byte(0x3F00+(sld->indexBits|colB))];
    }
    
    // sprites
    if((ppu.MASK & 0x10) != 0 && (!isLeftEdge || (ppu.MASK&0x4) != 0))
    {

        Eval_Sprite *sprites = sld->sprites;
        for(int i = 0; i < sld->spriteCount; i++)
        {
            u16 spriteTblAdr;
            if((ppu.CTRL&0x20) == 0)
                spriteTblAdr = ppu.spriteTblAdr;
            else
                spriteTblAdr = (sprites[i].idx&1)==0?0x0000:0x1000;
            if(x >= sprites[i].xPos && x < sprites[i].xPos+8)
            {
                u32 yOfst = (y-1)-(sprites[i].yPos);
                u32 xOfst = x-sprites[i].xPos;
                ///assert(yOfst >= 0 && yOfst < 8); // y not on screen
                ///assert(xOfst >= 0 && xOfst < 8); // x not on screen
                u8 vFlip = yOfst;
                if((0x80 & sprites[i].attr) != 0)
                    vFlip = 7 - yOfst;
                u8 slowSR = ppu_read_byte((sprites[i].idx<<4)+vFlip+spriteTblAdr);
                u8 shighSR = ppu_read_byte((sprites[i].idx<<4)+vFlip+spriteTblAdr+8);
                u8 sft=7-xOfst;
                if((0x40 & sprites[i].attr) != 0) // horizontal flip
                    sft = xOfst;
                u8 scolB = ((slowSR >> sft)&1) | (((shighSR >>sft)&1)<<1);
                u8 palBits = (sprites[i].attr & 0x3)<<2;
                u8 colIndex = ppu_read_byte(0x3F10 + (palBits | scolB));
                // background not opaque
                if(colB == 0 && scolB != 0) 
                    bgColor = palette[colIndex];
                // front sprite, always above bg
                else if((sprites[i].attr & 0x20) == 0 && scolB != 0)
                    bgColor = palette[colIndex];
                // sprite0 hit
                if(scolB != 0 && colB != 0 && sprites[i].isSprite0 && col != 255)
                {
                    ppu.STATUS |= 0x40;
                }
            }
        }
    }
    ///assert(x >= 0 && x < 256);
    ///assert(y >= 0 && y < 240);
    //NCX:
    ///displayBuffer[y][x] = bgColor;
    ///tft.drawPixel(x,y,bgColor);

    ILIbuffer[(y*256 + x) & (4095)] = bgColor;

  if(y == 16 && x==0) tft.drawImage(ILIbuffer, 0, 0, 256, 16);
  if(y == 32 && x==0) tft.drawImage(ILIbuffer, 0, 16, 256, 16);
  if(y == 48 && x==0) tft.drawImage(ILIbuffer, 0, 32, 256, 16);
  if(y == 64 && x==0) tft.drawImage(ILIbuffer, 0, 48, 256, 16);

  if(y == 80 && x==0) tft.drawImage(ILIbuffer, 0, 64, 256, 16);
  if(y == 96 && x==0) tft.drawImage(ILIbuffer, 0, 80, 256, 16);
  if(y == 112 && x==0) tft.drawImage(ILIbuffer, 0, 96, 256, 16);
  if(y == 128 && x==0) tft.drawImage(ILIbuffer, 0, 112, 256, 16);

  if(y == 144 && x==0) tft.drawImage(ILIbuffer, 0, 128, 256, 16);
  if(y == 160 && x==0) tft.drawImage(ILIbuffer, 0, 144, 256, 16);
  if(y == 176 && x==0) tft.drawImage(ILIbuffer, 0, 160, 256, 16);
  if(y == 192 && x==0) tft.drawImage(ILIbuffer, 0, 176, 256, 16);

  if(y == 208 && x==0) tft.drawImage(ILIbuffer, 0, 192, 256, 16);
  if(y == 224 && x==0) tft.drawImage(ILIbuffer, 0, 208, 256, 16);
  if(y == 240 && x==0) tft.drawImage(ILIbuffer, 0, 224, 256, 16);
       
}


b32 ppu_cycle() 
{
    b32 ret = FALSE;
    u32 slClk = ppu.clk%341;
    if(ppu.scanline == -1 && slClk == 1)
    {
        ppu.STATUS &= ~0x80; // clear VBLANK bit
        ppu.STATUS &= ~0x40; // clear Sprite0Hit
    }

    if(ppu.scanline >= 0 && ppu.scanline < 240 && slClk > 0 && slClk < 257)
        ppu_render_pixel(ppu.scanline, slClk-1);
    else if(slClk == 257)
        ppu_eval_sprites(ppu.scanline);         

    if(ppu.scanline == 241 && slClk == 1) // vblank start
    {
        ppu.STATUS |= 0x80; // set VBLANK bit
        if((ppu.CTRL & 0x80) != 0) // trigger NMI if NMI enabled
            cpu_nmi_trigger();
    }

    if(slClk == 340) // last clock for this scanline
    {
        if(ppu.scanline == 261) // last scanline
        {
            ppu.scanline = -1;
            ret = TRUE;
        }
        else
            ppu.scanline++;
    }
    ppu.clk++;
    return ret;
}

 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//apu.c


u8 lengthTable[32] =
{
    10,254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};

u8 pulseDuty[4][8] = 
{
    { 0, 1, 0, 0, 0, 0, 0, 0 },
    { 0, 1, 1, 0, 0, 0, 0, 0 },
    { 0, 1, 1, 1, 1, 0, 0, 0 },
    { 1, 0, 0, 1, 1, 1, 1, 1 }
};

u8 triangleTable[32] =
{
    15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
};

u16 ntscNoisePeriod[16] =  {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};

u16 palNoisePeriod[16] = {
    4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708,  944, 1890, 3778
};

u32 started;

r64 p1AmpProg = 0.0;
r64 p2AmpProg = 0.0;
r64 tAmpProg = 0.0;
r64 nAmpProg = 0.0;

// callback from platform layer asking for more audio
void AudioCallback(void *userdata, u8 *stream, i32 len)
{
    if(!started)
        return;

    i16* stream16 = (i16*)stream;
    u32 len16 = len/2;

    u8 p1Ctrl = apu.PULSE1[0];
    u8 p2Ctrl = apu.PULSE2[0];
    u8 nCtrl = apu.NOISE[0];
    u16 p1Vol = (p1Ctrl & 0x10) == 0 ? apu.p1Env.decay * 600 : (p1Ctrl & 0x0F) * 600;
    u16 p2Vol = (p2Ctrl & 0x10) == 0 ? apu.p2Env.decay * 600 : (p2Ctrl & 0x0F) * 600;
    u16 nVol = (nCtrl & 0x10) == 0 ? apu.nEnv.decay*2 : (nCtrl & 0x0F)*2;
    
    u16 p1T = apu.PULSE1[2] | ((apu.PULSE1[3]&0x7)<<8); // timer value
    u32 p1Freq = cpu.clockHz / (16*(p1T+1)); // fequency
    u32 p1P = cpu.clockHz/(16*p1Freq) - 1;

    u16 p2T = apu.PULSE2[2] | ((apu.PULSE2[3]&0x7)<<8); // timer value
    u32 p2Freq = cpu.clockHz / (16*(p2T+1)); // fequency
    u32 p2P = cpu.clockHz/(16*p2Freq) - 1;

    u16 tT = apu.TRIANGLE[2] | ((apu.TRIANGLE[3]&0x7)<<8); // timer value
    u32 tF = cpu.clockHz / (32*(tT+1)); // fequency

    u16 nT;
    if(emu.tvSystem == TV_SYSTEM_NTSC)
        nT = ntscNoisePeriod[apu.NOISE[2]&0xF];
    else
        nT = palNoisePeriod[apu.NOISE[2]&0xF];
    u32 nF = cpu.clockHz / (16*(nT+1));
    
    u32 p1Enabled = p1P >= 8 && apu.p1Length != 0 && (apu.CONTROL & 1) != 0;
    u32 p2Enabled = p2P >= 8 && apu.p2Length != 0 && (apu.CONTROL & 2) != 0;
    u32 tEnabled = apu.tLinCounter > 0 && apu.tLength > 0; 
    u32 nEnabled = apu.nLength != 0; // TODO: apu.CONTROL & 0x4?
    // TODO: DMC channel

    if((apu.CONTROL & 0x1) != 0) // pulse 1 enabled
    {
        r64 p1Amp = 1.0 / p1Freq;
        r64 p2Amp = 1.0 / p2Freq;
        r64 tAmp = 1.0 / tF;
        r64 nAmp = 1.0 / nF;

        r64 sAmp = 1.0 / SAMPLES_PER_SECOND;
        for(int i = 0; i < len16; i++)
        {
            p1AmpProg += sAmp/p1Amp;
            // modulus 0.999 because (unlikely) 1.0 would overflow duty index
            p1AmpProg = fmod(p1AmpProg, 0.999); 

            p2AmpProg += sAmp/p2Amp;
            p2AmpProg = fmod(p2AmpProg, 0.999);
            
            tAmpProg += sAmp/tAmp;
            tAmpProg = fmod(tAmpProg, 0.999);

            nAmpProg += sAmp/nAmp;
            if(nAmpProg/0.999 >= 0.999)
            {
                u8 otherBit;
                if((apu.NOISE[2]&0x80) != 0)
                    otherBit = (apu.nShift&0x40)>>6;
                else
                    otherBit = (apu.nShift&0x02)>>1;
                u8 feedback = apu.nShift&0x1 ^ otherBit;
                apu.nShift >>= 1;
                if(feedback)
                    apu.nShift |= 0x4000;
                else
                    apu.nShift &= ~0x4000;
            }
            nAmpProg = fmod(nAmpProg, 0.999);

            u32 dty1Idx = (u32)(p1AmpProg * 8);
            u32 dty2Idx = (u32)(p2AmpProg * 8);
            u32 triIdx = (u32)(tAmpProg * 32);

            // TODO: proper mixing and volume
            stream16[i] = 0;
            if(p1Enabled) // PULSE1
                stream16[i] += pulseDuty[p1Ctrl >> 6][dty1Idx]*p1Vol;
            if(p2Enabled) // PUSLE2
                stream16[i] += pulseDuty[p2Ctrl >> 6][dty2Idx]*p2Vol;
            if(tEnabled) // TRIANGLE
                stream16[i] += (triangleTable[triIdx]-7)*1200;
            if(nEnabled) // NOISE
                // TODO: replace totally random division
                stream16[i] += (apu.nShift/86)*nVol;
        }
    }
}

void apu_set_register(u8 reg, u8 value)
{
    switch(reg)
    {
        case 0x01:
        {
            apu.p1Sweep.reload = 1;
            apu.PULSE1[1] = value;
        } break;
        case 0x03: // pulse1 length/timerhigh
        {
            if((apu.CONTROL & 0x1) != 0) // pulse1 enabled
                apu.p1Length = lengthTable[value>>3];
            apu.p1Env.startFlag = 1;
            // TODO: restart envelope
            // TODO: reset phase of pulse generator (duty?)
            apu.PULSE1[3] = value;
        } break;
        case 0x05:
        {
            apu.p2Sweep.reload = 1;
            apu.PULSE2[1] = value;
        } break;
        case 0x07: // pulse2 length/timerhigh
        {
            if((apu.CONTROL & 0x2) != 0) // pulse2 enabled
                apu.p2Length = lengthTable[value>>3];
            apu.p2Env.startFlag = 1;
            // TODO: restart envelope
            // TODO: reset phase of pulse generator (duty?)
            apu.PULSE2[3] = value;
        } break;
        case 0x0B: // triangle length/timerhigh
        {
           // TODO: should this if be here?
           if((apu.CONTROL & 0x4) != 0)
               apu.tLength = lengthTable[value>>3];
            apu.tLinReloadFlag = 1; 
            apu.TRIANGLE[3] = value;
        } break;
        case 0x0F: // noise length
        {
            if((apu.CONTROL & 0x8) != 0)
                apu.nLength = lengthTable[value>>3];
            apu.nEnv.startFlag = 1;
            apu.NOISE[3] = value;
        } break;
        case 0x15: // APU CTRL
        {
            if((value & 0x1) == 0) // p1 enable bit
                apu.p1Length = 0;
            if((value & 0x2) == 0) // p2 enable bit
                apu.p2Length = 0;
            apu.CONTROL = value;
        } break;
        case 0x17: // FRAME_COUNTER 
        {
            // TODO: reset frame counter on write
            // with bit 7 set ($80) will immediately clock all of its controlled units at the beginning of the 5-step sequence (wtf does this even mean)
            apu.FRAME_COUNTER = value;
            apu.frameClkCount = 0;
            apu.frameCounter = 0;
            if((apu.FRAME_COUNTER & 0x80) != 0)
            {
                apu_half_frame();
                apu_quarter_frame();
            }
        } break;
        default:
        {
            *((u8*)&apu + reg) = value;
        } break;
    }
}

static inline void apu_length_tick()
{
    if((apu.PULSE1[0] & 0x20) == 0 && apu.p1Length > 0) // length halt bit clear
        apu.p1Length--;
    if((apu.PULSE2[0] & 0x20) == 0 && apu.p2Length > 0)
        apu.p2Length--;
    if((apu.TRIANGLE[0] & 0x80) == 0 && apu.tLength > 0)
        apu.tLength--;
    if((apu.NOISE[0] & 0x20) == 0 && apu.nLength > 0)
        apu.nLength--;
}

static inline void sweep(SweepUnit *su, u8 chReg[], u32 channel)
{
    u16 shiftAmount = ((chReg[1])&0x7);
    u16 timer = (chReg[2] | ((chReg[3]&0x07)<<8));
    u16 changeAmount = timer >> shiftAmount;
    u16 targetPeriod;
    if((chReg[1] & 0x08) != 0)
    {
        targetPeriod = timer - changeAmount;
        if(channel == 1)
            targetPeriod--;
    }
    else
    {
        targetPeriod = timer + changeAmount;
    }
    if(targetPeriod <= 0x7FF && targetPeriod >= 8)
    {
        chReg[2] = targetPeriod & 0xFF;
        chReg[3] = (chReg[3] & ~0x7) | ((targetPeriod >> 8) & 0x7);
    } // TODO: else send 0 to mixer
}

static inline void sweep_tick(SweepUnit *su, u8 chReg[], u32 channel)
{
    u8 sweepCtrl = chReg[1];
    if(su->reload)
    {
        if(su->divider == 0 && (sweepCtrl & 0x80) != 0)
        {
            sweep(su, chReg, channel);
        }
        su->divider = (sweepCtrl >> 4) & 0x7;
        su->reload = 0;
    }
    else if(su->divider != 0)
        su->divider--;
    else
    {
        if((sweepCtrl & 0x80) != 0)
        {
            su->divider = (sweepCtrl >> 4) & 0x7;
            sweep(su, chReg, channel);
        }
    }
}

static inline void envelope_tick(EnvelopeUnit *env, u8 ctrlReg)
{
    if(!env->startFlag) // start flag clear, clock divider
    {
        if(env->divider == 0)
        {
            env->divider = ctrlReg&0x0F;
            if(env->decay != 0)
                env->decay--;
            else if((ctrlReg & 0x20) != 0) // looping enabled
                env->decay = 15;
        }
        else
            env->divider--;
    }
    else // start envelope divider
    {
        env->divider = ctrlReg&0x0F;
        env->decay = 15;
        env->startFlag = 0;
    }
}

// triangle channel linear counter
static inline void apu_linear_counter_tick()
{
    if(apu.tLinReloadFlag)
        apu.tLinCounter = apu.TRIANGLE[0] & 0x7F;
    else
    {
        if(apu.tLinCounter != 0)
            apu.tLinCounter--;
    }
    if((apu.TRIANGLE[0] & 0x80) == 0) // tri ctrl flag clear
        apu.tLinReloadFlag = 0;
}

static inline void apu_half_frame()
{
    apu_length_tick();
    sweep_tick(&apu.p1Sweep, apu.PULSE1, 1);
    sweep_tick(&apu.p2Sweep, apu.PULSE2, 2);
}

static inline void apu_quarter_frame()
{
    envelope_tick(&apu.p1Env, apu.PULSE1[0]);
    envelope_tick(&apu.p2Env, apu.PULSE2[0]);
    envelope_tick(&apu.nEnv, apu.NOISE[0]);
    apu_linear_counter_tick();
}

void apu_init()
{
    apu.nShift = 1;
}

// APU clock cycle at (CPU clock / 2)
void apu_cycle()
{
    // TODO: if CPU interrupt inhibit is set, don't set FC interrupt flag
    started = 1;

    if(apu.frameClkCount >= apu.clocksPerFrame) // 240Hz frame counter
    {
        if((apu.FRAME_COUNTER & 0x80) == 0) // 4 step mode
        {
            if(apu.frameCounter == 1 || apu.frameCounter == 3) // sweep and length counters (~120Hz)
            {
                apu_half_frame();
            }
            // TODO: interrupt (also see comment at the start of this function)
            apu_quarter_frame(); // full ~240Hz
            if(apu.frameCounter >= 3)
                apu.frameCounter = 0;
        }
        else // 5 step mode
        {
            if(apu.frameCounter == 1 || apu.frameCounter == 4) // sweep and length counters (~96Hz)
            {
                apu_half_frame();
            }
            // TODO: interrupt
            if(apu.frameCounter != 3) // ~192Hz
                apu_quarter_frame();

            if(apu.frameCounter >= 4)
                apu.frameCounter = 0;
        }
        apu.frameCounter++;
        apu.frameClkCount = 0;
    }
    apu.frameClkCount++;
}
 


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//cpu.c





#define OP_STP 0x01
#define OP_NOP 0x02

#define OP_LDY 0x11
#define OP_JMP 0x12
#define OP_CLI 0x13
#define OP_SEI 0x14
#define OP_BIT 0x15
#define OP_CPX 0x16
#define OP_CPY 0x17
#define OP_PHP 0x18
#define OP_PLP 0x19
#define OP_DEY 0x1A
#define OP_TAY 0x1B
#define OP_INY 0x1C
#define OP_TYA 0x1D
#define OP_STY 0x1E
#define OP_PLA 0x1F
#define OP_RTS 0x20
#define OP_BVC 0x21
#define OP_PHA 0x22
#define OP_RTI 0x23
#define OP_SEC 0x24
#define OP_BMI 0x25
#define OP_JSR 0x26
#define OP_CLC 0x27
#define OP_BPL 0x28
#define OP_BRK 0x29
#define OP_BVS 0x2A
#define OP_BCC 0x2B
#define OP_SHY 0x2C
#define OP_BCS 0x2D
#define OP_CLV 0x2E
#define OP_BNE 0x2F
#define OP_CLD 0x30
#define OP_INX 0x31
#define OP_BEQ 0x32
#define OP_SED 0x33

#define OP_EOR 0x41
#define OP_AND 0x42
#define OP_ORA 0x43
#define OP_ADC 0x44
#define OP_STA 0x45
#define OP_LDA 0x46
#define OP_CMP 0x47
#define OP_SBC 0x48

#define OP_INC 0x60
#define OP_DEC 0x61
#define OP_LDX 0x62
#define OP_TSX 0x63
#define OP_TXS 0x64
#define OP_TXA 0x65
#define OP_ROR 0x66
#define OP_STX 0x67
#define OP_LSR 0x68
#define OP_ROL 0x69
#define OP_ASL 0x6A
#define OP_SHX 0x6B
#define OP_TAX 0x6C
#define OP_DEX 0x6D // 59 official

#define OP_DCP 0x71
#define OP_LAS 0x72
#define OP_LAX 0x73
#define OP_SAX 0x74
#define OP_RRA 0x75
#define OP_SRE 0x76
#define OP_RLA 0x77
#define OP_SLO 0x78
#define OP_ISC 0x79
#define OP_AHX 0x7A
#define OP_ARR 0x7B
#define OP_ALR 0x7C
#define OP_ANC 0x7D
#define OP_XAA 0x7E
#define OP_TAS 0x7F
#define OP_AXS 0x70 // 75 with unofficial

#define ADR_MODE_IMPLIED        0x0
#define ADR_MODE_IMMEDIATE      0x1
#define ADR_MODE_INDEXED_INDIR  0x2
#define ADR_MODE_ZP             0x3
#define ADR_MODE_ABS            0x4
#define ADR_MODE_INDIR_INDEXED  0x5
#define ADR_MODE_ZP_INDEXED_X   0x6
#define ADR_MODE_ZP_INDEXED_Y   0x7
#define ADR_MODE_ABS_INDEXED_X  0x8
#define ADR_MODE_ABS_INDEXED_Y  0x9
#define ADR_MODE_PC_RELATIVE    0xA
#define ADR_MODE_JMP_INDIR      0xB

#define FETCH_PC_BYTE() (read_byte(cpu.PC++))

#define SET_SIGN_FLAG_VAL(x) (cpu.signFlag = ((x)&0x80)!=0)
#define SET_ZERO_FLAG_VAL(x) (cpu.zeroFlag = ((x)==0))

static inline void PUSH_BYTE(u8 b)
{
    u16 adr = 0x100 | cpu.S;
    write_byte(adr, b);
    cpu.S--;
}

static inline u8 POP_BYTE()
{
    u8 prev = ++cpu.S;
    u16 adr = 0x100|prev;
    u8 b = read_byte(adr);
    return b;
}

const char* adr_mode_to_string(u8 adrMode)
{
    switch(adrMode)
    {
    case ADR_MODE_IMPLIED:
        return "ADR_MODE_IMPLIED";
    case ADR_MODE_IMMEDIATE:
        return "ADR_MODE_IMMEDIATE";
    case ADR_MODE_INDEXED_INDIR:
        return "ADR_MODE_INDEXED_INDIR";
    case ADR_MODE_ZP:
        return "ADR_MODE_ZP";
    case ADR_MODE_ABS:
        return "ADR_MODE_ABS";
    case ADR_MODE_INDIR_INDEXED:
        return "ADR_MODE_INDIR_INDEXED";
    case ADR_MODE_ZP_INDEXED_X:
        return "ADR_MODE_ZP_INDEXED_X";
    case ADR_MODE_ZP_INDEXED_Y:
        return "ADR_MODE_ZP_INDEXED_Y";
    case ADR_MODE_ABS_INDEXED_X:
        return "ADR_MODE_ABS_INDEXED_X";
    case ADR_MODE_ABS_INDEXED_Y:
        return "ADR_MODE_ABS_INDEXED_Y";
    case ADR_MODE_PC_RELATIVE:
        return "ADR_MODE_PC_RELATIVE";
    case ADR_MODE_JMP_INDIR:
        return "ADR_MODE_JMP_INDIR";
    default:
        return "ADR_MODE_UNKNOWN";
    }
}

const char* inst_to_string(u8 op)
{
    switch(op)
    {
        case OP_STP:
            return "STP";
        case OP_NOP:
            return "NOP";
        case OP_LDY:
            return "LDY";
        case OP_JMP:
            return "JMP";
        case OP_CLI:
            return "CLI";
        case OP_SEI:
            return "SEI";
        case OP_BIT:
            return "BIT";
        case OP_CPX:
            return "CPX";
        case OP_CPY:
            return "CPY";
        case OP_PHP:
            return "PHP";
        case OP_PLP:
            return "PLP";
        case OP_DEY:
            return "DEY";
        case OP_TAY:
            return "TAY";
        case OP_INY:
            return "INY";
        case OP_TYA:
            return "TYA";
        case OP_STY:
            return "STY";
        case OP_PLA:
            return "PLA";
        case OP_RTS:
            return "RTS";
        case OP_BVC:
            return "BVC";
        case OP_PHA:
            return "PHA";
        case OP_RTI:
            return "RTI";
        case OP_SEC:
            return "SEC";
        case OP_BMI:
            return "BMI";
        case OP_JSR:
            return "JSR";
        case OP_CLC:
            return "CLC";
        case OP_BPL:
            return "BPL";
        case OP_BRK:
            return "BRK";
        case OP_BVS:
            return "BVS";
        case OP_BCC:
            return "BCC";
        case OP_SHY:
            return "SHY";
        case OP_BCS:
            return "BCS";
        case OP_CLV:
            return "CLV";
        case OP_BNE:
            return "BNE";
        case OP_CLD:
            return "CLD";
        case OP_INX:
            return "INX";
        case OP_BEQ:
            return "BEQ";
        case OP_SED:
            return "SED";

        case OP_EOR:
            return "EOR";
        case OP_AND:
            return "AND";
        case OP_ORA:
            return "ORA";
        case OP_ADC:
            return "ADC";
        case OP_STA:
            return "STA";
        case OP_LDA:
            return "LDA";
        case OP_CMP:
            return "CMP";
        case OP_SBC:
            return "SBC";
        case OP_INC:
            return "INC";
        case OP_DEC:
            return "DEC";
        case OP_LDX:
            return "LDX";
        case OP_TSX:
            return "TSX";
        case OP_TXS:
            return "TXS";
        case OP_TXA:
            return "TXA";
        case OP_ROR:
            return "ROR";
        case OP_STX:
            return "STX";
        case OP_LSR:
            return "LSR";
        case OP_ROL:
            return "ROL";
        case OP_ASL:
            return "ASL";
        case OP_SHX:
            return "SHX";
        case OP_TAX:
            return "TAX";
        case OP_DEX:
            return "DEX";
        case OP_DCP:
            return "DCP";
        case OP_LAS:
            return "LAS";
        case OP_LAX:
            return "LAX";
        case OP_SAX:
            return "SAX";
        case OP_RRA:
            return "RRA";
        case OP_SRE:
            return "SRE";
        case OP_RLA:
            return "RLA";
        case OP_SLO:
            return "SLO";
        case OP_ISC:
            return "ISC";
        case OP_AHX:
            return "AHX";
        case OP_ARR:
            return "ARR";
        case OP_ALR:
            return "ALR";
        case OP_ANC:
            return "ANC";
        case OP_XAA:
            return "XAA";
        case OP_TAS:
            return "TAS";
        case OP_AXS:
            return "AXS";
    }
    return "UNKNOWN";
}

u8 decode_table[256] = 
{
    OP_BRK, OP_ORA, OP_STP, OP_SLO, OP_NOP, OP_ORA, OP_ASL, OP_SLO, 
    OP_PHP, OP_ORA, OP_ASL, OP_ANC, OP_NOP, OP_ORA, OP_ASL, OP_SLO, 
    OP_BPL, OP_ORA, OP_STP, OP_SLO, OP_NOP, OP_ORA, OP_ASL, OP_SLO,
    OP_CLC, OP_ORA, OP_NOP, OP_SLO, OP_NOP, OP_ORA, OP_ASL, OP_SLO,
    
    OP_JSR, OP_AND, OP_STP, OP_RLA, OP_BIT, OP_AND, OP_ROL, OP_RLA, 
    OP_PLP, OP_AND, OP_ROL, OP_ANC, OP_BIT, OP_AND, OP_ROL, OP_RLA, 
    OP_BMI, OP_AND, OP_STP, OP_RLA, OP_NOP, OP_AND, OP_ROL, OP_RLA,
    OP_SEC, OP_AND, OP_NOP, OP_RLA, OP_NOP, OP_AND, OP_ROL, OP_RLA,
    
    OP_RTI, OP_EOR, OP_STP, OP_SRE, OP_NOP, OP_EOR, OP_LSR, OP_SRE, 
    OP_PHA, OP_EOR, OP_LSR, OP_ALR, OP_JMP, OP_EOR, OP_LSR, OP_SRE, 
    OP_BVC, OP_EOR, OP_STP, OP_SRE, OP_NOP, OP_EOR, OP_LSR, OP_SRE,
    OP_CLI, OP_EOR, OP_NOP, OP_SRE, OP_NOP, OP_EOR, OP_LSR, OP_SRE,
    
    OP_RTS, OP_ADC, OP_STP, OP_RRA, OP_NOP, OP_ADC, OP_ROR, OP_RRA, 
    OP_PLA, OP_ADC, OP_ROR, OP_ARR, OP_JMP, OP_ADC, OP_ROR, OP_RRA, 
    OP_BVS, OP_ADC, OP_STP, OP_RRA, OP_NOP, OP_ADC, OP_ROR, OP_RRA,
    OP_SEI, OP_ADC, OP_NOP, OP_RRA, OP_NOP, OP_ADC, OP_ROR, OP_RRA,
    
    OP_NOP, OP_STA, OP_NOP, OP_SAX, OP_STY, OP_STA, OP_STX, OP_SAX, 
    OP_DEY, OP_NOP, OP_TXA, OP_XAA, OP_STY, OP_STA, OP_STX, OP_SAX, 
    OP_BCC, OP_STA, OP_STP, OP_AHX, OP_STY, OP_STA, OP_STX, OP_SAX,
    OP_TYA, OP_STA, OP_TXS, OP_TAS, OP_SHY, OP_STA, OP_SHX, OP_AHX,
    
    OP_LDY, OP_LDA, OP_LDX, OP_LAX, OP_LDY, OP_LDA, OP_LDX, OP_LAX, 
    OP_TAY, OP_LDA, OP_TAX, OP_LAX, OP_LDY, OP_LDA, OP_LDX, OP_LAX, 
    OP_BCS, OP_LDA, OP_STP, OP_LAX, OP_LDY, OP_LDA, OP_LDX, OP_LAX,
    OP_CLV, OP_LDA, OP_TSX, OP_LAS, OP_LDY, OP_LDA, OP_LDX, OP_LAX,
    
    OP_CPY, OP_CMP, OP_NOP, OP_DCP, OP_CPY, OP_CMP, OP_DEC, OP_DCP, 
    OP_INY, OP_CMP, OP_DEX, OP_AXS, OP_CPY, OP_CMP, OP_DEC, OP_DCP, 
    OP_BNE, OP_CMP, OP_STP, OP_DCP, OP_NOP, OP_CMP, OP_DEC, OP_DCP,
    OP_CLD, OP_CMP, OP_NOP, OP_DCP, OP_NOP, OP_CMP, OP_DEC, OP_DCP,
    
    OP_CPX, OP_SBC, OP_NOP, OP_ISC, OP_CPX, OP_SBC, OP_INC, OP_ISC, 
    OP_INX, OP_SBC, OP_NOP, OP_SBC, OP_CPX, OP_SBC, OP_INC, OP_ISC, 
    OP_BEQ, OP_SBC, OP_STP, OP_ISC, OP_NOP, OP_SBC, OP_INC, OP_ISC,
    OP_SED, OP_SBC, OP_NOP, OP_ISC, OP_NOP, OP_SBC, OP_INC, OP_ISC
};


u8 adr_mode_table[256] =
{
    0,  2,  0,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  6,  6,  0,  9,  0,  9,  8,  8,  8,  8,

    4,  2,  0,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  6,  6,  0,  9,  0,  9,  8,  8,  8,  8,

    0,  2,  0,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  6,  6,  0,  9,  0,  9,  8,  8,  8,  8,

    0,  2,  0,  2,  3,  3,  3,  3,  0,  1,  0,  1, 11,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  6,  6,  0,  9,  0,  9,  8,  8,  8,  8,

    1,  2,  1,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  7,  7,  0,  9,  0,  9,  8,  8,  8,  8,

    1,  2,  1,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  7,  7,  0,  9,  0,  9,  8,  8,  9,  8,

    1,  2,  1,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  6,  6,  0,  9,  0,  9,  8,  8,  8,  8,

    1,  2,  1,  2,  3,  3,  3,  3,  0,  1,  0,  1,  4,  4,  4,  4,
   10,  5,  0,  5,  6,  6,  6,  6,  0,  9,  0,  9,  8,  8,  8,  8
};

u8 clock_table[256] = 
{
    7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
  2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
  2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
  2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
  2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
  2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
  2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
  2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
  2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7
};

static u32 clk;

u8 read_byte(u16 adr)
{
    u8 ret;
    if(adr >= 0x0000 && adr < 0x2000) // internal 2KB RAM
    {
        ret = memory[adr & 0x7FF];
    }
    else if(adr >= 0x2000 && adr < 0x4000) // ppu registers
    {
        ret = ppu_get_register(adr&0x7);
    }
    else if(adr >= 0x4000 && adr < 0x4020) // APU and IO
    {
        switch(adr)
        {
            case 0x4014:
                ///assert(FALSE);
                break;
            case 0x4015:
                ret = apu.CONTROL;
                apu.FRAME_COUNTER &= ~0x40;
                break;
            case 0x4016: // JP 1
                ret = cpu.pad1Data;
                cpu.pad1Data <<= 1;
                ret = (ret>>7)&1;
                break;
            case 0x4018: // JP 2
                // TODO: implement
                ret = 0;
                break;
            default:
                ret = *((u8*)&apu + (adr - 0x4000));
                break;
        }
    }
    else if(adr >= 4020) // cartridge
    {
        //NCX:
        return  memory[adr&0xFFFF]; ///emu.crRead(adr);
    }
    return ret;
}

void write_byte(u16 adr, u8 value)
{
    if(adr >= 0x0000 && adr < 0x2000)
        memory[adr&0x7FF] = value;
    else if(adr >= 0x2000 && adr < 0x4000) // PPU
        ppu_set_register(adr&0x7, value);
    else if(adr >= 0x4000 && adr < 0x4020) // APU and IO registers
    {
        switch(adr)
        {
            case 0x4014: // OAM DMA
                cpu.suspClocks = 514; // suspend CPU for 514 cycles
                // write of 0xXX will copy 0xXX00-0xXXFF to PPU SPR-RAM from CPU Page
                for(u32 i = 0; i <= 0xFF; i++)
                {
                    u16 adr = (value<<8) | (i&0xFF);
                    u8 val = read_byte(adr);
                    ppu_spr_ram[i] = val;
                }
                break;
            case 0x4016: // joystick strobe
                ///cpu.strobe = value;
                break;
            default:
                ///apu_set_register(adr&0x1F, value);
                break;
        }
    }
    else if(adr >= 0x4020) // cartridge
        ///emu.crWrite(adr, value);
        memory[adr&0xFFFF]=value;
    else {
        ///assert(FALSE);
    }    
}

static inline u8 cpu_get_status()
{
    return (cpu.carryFlag) | (cpu.zeroFlag<<1) | (cpu.intFlag<<2) | (cpu.decimalFlag<<3) | (cpu.overflowFlag<<6) | (cpu.signFlag<<7) | 0x20;
}

static inline void cpu_set_status(u8 status)
{
    cpu.carryFlag = status&0x1;
    cpu.zeroFlag = (status&0x2)>>1;
    cpu.intFlag = (status&0x4)>>2;
    cpu.decimalFlag = (status&0x8)>>3;
    cpu.overflowFlag = (status&0x40)>>6;
    cpu.signFlag = (status&0x80)>>7;
}

void cpu_nmi_trigger()
{
    u16 from = cpu.PC;
    PUSH_BYTE((cpu.PC>>8)&0xFF);
    PUSH_BYTE(cpu.PC&0xFF);
    PUSH_BYTE(cpu_get_status());
    cpu.intFlag = 1;
    u16 jmpLoc = (read_byte(0xFFFA) | (read_byte(0xFFFB) << 8));
    cpu.PC = jmpLoc;
}

void cpu_cycle()
{
    u8 op, instr, adrMode, src;
    u16 adr;

    if(cpu.suspClocks > 0) // if clocks left from previous instruction then do nothing
        goto cpu_end_cycle;

    if((cpu.strobe & 1) != 0) // if controller strobing enabled then strobe them
    {
        cpu.pad1Data = buttons;
        cpu.pad2Data = 0;
    }

    op = FETCH_PC_BYTE();
    instr = decode_table[op];
    adrMode = adr_mode_table[op];
    cpu.suspClocks = clock_table[op];

#define GET_SRC_VAL() ( adrMode == ADR_MODE_IMMEDIATE || adrMode == ADR_MODE_IMPLIED ? src : read_byte(adr))

    //printf("Execute %s(0x%02x) %s clk: %d\n", inst_to_string(instr), op, adr_mode_to_string(adrMode), clk);

    switch(adrMode)
    {
        case ADR_MODE_IMPLIED:
            src = cpu.A;
            read_byte(cpu.PC); // dummy read for emulation
            break;
        case ADR_MODE_ZP:
            {
                adr = FETCH_PC_BYTE();
            } break;
        case ADR_MODE_IMMEDIATE:
            src = FETCH_PC_BYTE();
            break;
        case ADR_MODE_ABS:
            {
                u8 adrlow = FETCH_PC_BYTE();
                u8 adrhigh = FETCH_PC_BYTE();
                adr = (adrhigh << 8) | adrlow;
            } break;
        case ADR_MODE_ZP_INDEXED_X:
            {
                adr = FETCH_PC_BYTE();
                adr = ((adr+cpu.X)&0xFF);
            } break;
        case ADR_MODE_ZP_INDEXED_Y:
            adr = FETCH_PC_BYTE();
            adr = ((adr+cpu.Y)&0xFF);
            break;
        case ADR_MODE_ABS_INDEXED_X:
            {
                u8 adrlow = FETCH_PC_BYTE();
                u8 adrhigh = FETCH_PC_BYTE();
                adr = (adrhigh << 8) | adrlow;
                adr += cpu.X;
            } break;
        case ADR_MODE_ABS_INDEXED_Y:
            {
                u8 adrlow = FETCH_PC_BYTE();
                u8 adrhigh = FETCH_PC_BYTE();
                adr = (adrhigh << 8) | adrlow;
                adr += cpu.Y;
            } break;
        case ADR_MODE_PC_RELATIVE:
            {
                u8 rel = FETCH_PC_BYTE();
                adr = cpu.PC;
                if(rel & 0x80)
                    adr += (rel-0x100);
                else
                    adr += rel;
            } break;
        case ADR_MODE_INDEXED_INDIR:
            {
                u8 adrl = FETCH_PC_BYTE();
                u8 f = read_byte((adrl+cpu.X)&0xFF);
                u8 s = read_byte((adrl+cpu.X+1)&0xFF);
                adr = f+(s<<8);
            } break;
        case ADR_MODE_INDIR_INDEXED:
            {
                u8 adrl = FETCH_PC_BYTE();
                u8 f = read_byte(adrl);
                u8 s = read_byte((adrl+1)&0xFF);
                adr = f+(s<<8)+cpu.Y;
            } break;
        case ADR_MODE_JMP_INDIR:
            {
                u8 adrlow = FETCH_PC_BYTE();
                u8 adrhigh = FETCH_PC_BYTE();
                adr = (adrhigh << 8) | adrlow;
                adrlow = read_byte(adr);
                adrhigh = read_byte(adr+1);
                adr = (adrhigh << 8) | adrlow;
            } break;
        default:
            //////assert(FALSE); // unknown addressing mode
            break;
    }

    switch(instr)
    {
        case OP_SEI:
            cpu.intFlag = 1;
            break;
        case OP_CLI:
            cpu.intFlag = 0;
            break;
        case OP_SED:
            cpu.decimalFlag = 1;
            break;
        case OP_CLD:
            cpu.decimalFlag = 0;
            break;
        case OP_CLV:
            cpu.overflowFlag = 0;
            break;
        case OP_SEC:
            cpu.carryFlag = 1;
            break;
        case OP_CLC:
            cpu.carryFlag = 0;
            break;
        case OP_INX:
            cpu.X++;
            SET_ZERO_FLAG_VAL(cpu.X);
            SET_SIGN_FLAG_VAL(cpu.X);
            break;
        case OP_INY:
            cpu.Y++;
            SET_ZERO_FLAG_VAL(cpu.Y);
            SET_SIGN_FLAG_VAL(cpu.Y);
            break;
        case OP_DEC:
            src = GET_SRC_VAL();
            src -= 1;
            SET_ZERO_FLAG_VAL(src);
            SET_SIGN_FLAG_VAL(src);
            write_byte(adr, src);
            break;
        case OP_LDA:
            src = GET_SRC_VAL();
            cpu.A = src;
            SET_ZERO_FLAG_VAL(src);
            SET_SIGN_FLAG_VAL(src);
            break;
        case OP_BPL:
            if (!cpu.signFlag) {
                cpu.PC = adr;
            } break;
        case OP_BMI:
            if (cpu.signFlag) {
                cpu.PC = adr;
            } break;

        case OP_BCS:
            if (cpu.carryFlag) {
                cpu.PC = adr;
            } break;
        case OP_BCC:
            if (!cpu.carryFlag) {
                cpu.PC = adr;
            } break;

        case OP_BNE:
            if (!cpu.zeroFlag) {
                cpu.PC = adr;
            } break;
        case OP_BEQ:
            if(cpu.zeroFlag) {
                cpu.PC = adr;
            } break;
        case OP_BVS:
            if(cpu.overflowFlag) {
                cpu.PC = adr;
            } break;
        case OP_BVC:
            if(!cpu.overflowFlag) {
                cpu.PC = adr;
            } break;
        case OP_JMP:
            cpu.PC = adr;
            break;
        case OP_BRK:
            {
                cpu.PC++;
                PUSH_BYTE((cpu.PC>>8)&0xFF);
                PUSH_BYTE(cpu.PC&0xFF);
                PUSH_BYTE(cpu_get_status() | 0x10);
                cpu.intFlag = (1);
                u16 jmpLoc = (read_byte(0xFFFE) | (read_byte(0xFFFF) << 8));
                cpu.PC = jmpLoc;
            } break;
        case OP_RTI:
            {
                u8 val = POP_BYTE();
                cpu_set_status(val);
                u8 retLow = POP_BYTE();
                u8 retHigh = POP_BYTE();
                u16 retAdr = (retHigh << 8) | retLow;
                cpu.PC = retAdr;
            } break;
        case OP_ORA:
            src = GET_SRC_VAL();
            src |= cpu.A;
            SET_ZERO_FLAG_VAL(src);
            SET_SIGN_FLAG_VAL(src);
            cpu.A = src;
            break;
        case OP_NOP:
            break;
        case OP_ASL:
            if(adrMode == ADR_MODE_IMPLIED)
            {
                cpu.carryFlag = ((cpu.A & 0x80) != 0);
                cpu.A <<= 1;
                SET_SIGN_FLAG_VAL(cpu.A);
                SET_ZERO_FLAG_VAL(cpu.A);
            }
            else
            {
                src = GET_SRC_VAL();
                cpu.carryFlag = ((src & 0x80) != 0);
                u8 val = src << 1;
                write_byte(adr, val);
                SET_SIGN_FLAG_VAL(val);
                SET_ZERO_FLAG_VAL(val);
            }
            break;
        case OP_LSR:
            {
                if(adrMode == ADR_MODE_IMPLIED)
                {
                    cpu.carryFlag = ((cpu.A & 0x01) != 0);
                    cpu.A >>= 1;
                    SET_SIGN_FLAG_VAL(cpu.A);
                    SET_ZERO_FLAG_VAL(cpu.A);
                }
                else
                {
                    src = GET_SRC_VAL();
                    cpu.carryFlag = ((src & 0x01) != 0);
                    u8 val = src >> 1;
                    write_byte(adr, val);
                    SET_SIGN_FLAG_VAL(val);
                    SET_ZERO_FLAG_VAL(val);
                }

            } break;
       case OP_STA:
            write_byte(adr, cpu.A);
            break;
        case OP_STX:
            write_byte(adr, cpu.X);
            break;
        case OP_STY:
            write_byte(adr, cpu.Y);
            break;
        case OP_LDX:
            src = GET_SRC_VAL();
            cpu.X = src;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            break;
        case OP_LDY:
            src = GET_SRC_VAL();
            cpu.Y = src;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            break;
        case OP_CMP:
            {
                src = GET_SRC_VAL();
                u16 val = cpu.A - src;
                cpu.carryFlag = (val < 0x100);
                SET_SIGN_FLAG_VAL(val&0xFF);
                SET_ZERO_FLAG_VAL(val&0xFF);
            } break;
        case OP_CPX:
            {
                src = GET_SRC_VAL();
                u16 val = cpu.X - src;
                cpu.carryFlag = (val < 0x100);
                SET_SIGN_FLAG_VAL(val&0xFF);
                SET_ZERO_FLAG_VAL(val&0xFF);
            } break;
        case OP_CPY:
            {
                src = GET_SRC_VAL();
                u16 val = cpu.Y - src;
                cpu.carryFlag = (val < 0x100);
                SET_SIGN_FLAG_VAL(val&0xFF);
                SET_ZERO_FLAG_VAL(val&0xFF);
            } break;
        case OP_TXS:
            cpu.S = cpu.X;
            break;
        case OP_JSR:
            cpu.PC--;
            PUSH_BYTE((cpu.PC>>8)&0xFF);
            PUSH_BYTE(cpu.PC&0xFF);
            cpu.PC = adr;
            break;
        case OP_RTS:
            adr = POP_BYTE()&0xFF;
            adr += (POP_BYTE() << 8) + 1;
            cpu.PC = adr;
            break;
        case OP_DEY:
            src = cpu.Y - 1;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            cpu.Y = src;
            break;
        case OP_DEX:
            src = cpu.X - 1;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            cpu.X = src;
            break;
        case OP_BIT:
            src = GET_SRC_VAL();
            SET_SIGN_FLAG_VAL(src);
            cpu.overflowFlag = ((0x40 & src) != 0);
            SET_ZERO_FLAG_VAL(src & cpu.A);
            break;
        case OP_AND:
            src = GET_SRC_VAL();
            src &= cpu.A;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            cpu.A = src;
            break;
        case OP_EOR:
            src = GET_SRC_VAL();
            src ^= cpu.A;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            cpu.A = src;
            break;
        case OP_TXA:
            SET_SIGN_FLAG_VAL(cpu.X);
            SET_ZERO_FLAG_VAL(cpu.X);
            cpu.A = cpu.X;
            break;
        case OP_TYA:
            SET_SIGN_FLAG_VAL(cpu.Y);
            SET_ZERO_FLAG_VAL(cpu.Y);
            cpu.A = cpu.Y;
            break;
        case OP_TAX:
            SET_SIGN_FLAG_VAL(cpu.A);
            SET_ZERO_FLAG_VAL(cpu.A);
            cpu.X = cpu.A;
            break;
        case OP_TAY:
            SET_SIGN_FLAG_VAL(cpu.A);
            SET_ZERO_FLAG_VAL(cpu.A);
            cpu.Y = cpu.A;
            break;
        case OP_TSX:
            src = cpu.S;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            cpu.X = src;
            break;
        case OP_PHA:
            PUSH_BYTE(cpu.A);
            break;
        case OP_PLA:
            src = POP_BYTE();
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            cpu.A = src;
            break;
        case OP_INC:
            src = GET_SRC_VAL();
            src += 1;
            SET_SIGN_FLAG_VAL(src);
            SET_ZERO_FLAG_VAL(src);
            write_byte(adr, src);
            break;
        case OP_ROL:
            {
                u16 val = GET_SRC_VAL();
                val <<= 1;
                if(cpu.carryFlag)
                    val |= 0x01;
                cpu.carryFlag = (val > 0xff);
                SET_SIGN_FLAG_VAL(val&0xFF);
                SET_ZERO_FLAG_VAL(val&0xFF);
                if(adrMode == ADR_MODE_IMPLIED)
                    cpu.A = val&0xFF;
                else
                    write_byte(adr, val&0xFF);
            } break;
        case OP_ROR:
            {
                u16 val = GET_SRC_VAL();
                if(cpu.carryFlag)
                    val |= 0x100;
                cpu.carryFlag = (val & 0x01);
                val >>= 1;
                SET_SIGN_FLAG_VAL(val & 0xFF);
                SET_ZERO_FLAG_VAL(val & 0xFF);
                if(adrMode == ADR_MODE_IMPLIED)
                    cpu.A = val&0xFF;
                else
                    write_byte(adr, val&0xFF);
            } break;

        case OP_SBC:
            {
                src = GET_SRC_VAL();
                u16 val = cpu.A - src - (cpu.carryFlag ? 0 : 1);
                SET_SIGN_FLAG_VAL(val & 0xFF);
                SET_ZERO_FLAG_VAL(val & 0xFF);
                cpu.overflowFlag = ((((cpu.A ^ val) & 0x80) && ((cpu.A ^ src) & 0x80)));
                cpu.carryFlag = (val < 0x100);
                cpu.A = (val & 0xFF);

            } break;
        case OP_ADC:
            {
                src = GET_SRC_VAL();
                u16 val = src + cpu.A + (cpu.carryFlag ? 1 : 0);
                SET_ZERO_FLAG_VAL(val & 0xFF);
                SET_SIGN_FLAG_VAL(val&0xFF);
                cpu.overflowFlag = (!((cpu.A ^ src) & 0x80) && ((cpu.A ^ val) & 0x80));
                cpu.carryFlag = (val > 0xFF);
                cpu.A = val&0xFF;
            } break;
        case OP_PHP:
            PUSH_BYTE(cpu_get_status());
            break;
        case OP_PLP:
            src = POP_BYTE();
            cpu_set_status(src);
            break;
        default:
            fprintf(stderr, "Execute %s(0x%02x) %s clk: %d\n", inst_to_string(instr), op, adr_mode_to_string(adrMode), clk);
            fprintf(stderr, "Unknown instruction \n");
            ///assert(FALSE);
            break;
    }
    ///assert(cpu.suspClocks != 0);
cpu_end_cycle:
    cpu.suspClocks--;
    clk++;
    //printf("A: %02x X: %02x, Y: %02x, P: %02x adr:%04x src:%04x\n", (u32)cpu.A, (u32)cpu.X, (u32)cpu.Y, (u32)cpu.P, adr, src);
}
 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//nes.c (part)



void show_error(const char* title, const char* error)
{
    tft.print("Error: ");
    tft.print(title);
    tft.print("Internal: "); 
    tft.println(error);
}

void emu_set_nt_mirroring(u32 mirroring)
{
    switch(mirroring)
    {
        case NAMETABLE_MIRRORING_SINGLE_LOW:
            emu.currentNtPtr[0] = ppu_internal_ram;
            emu.currentNtPtr[1] = ppu_internal_ram;
            emu.currentNtPtr[2] = ppu_internal_ram;
            emu.currentNtPtr[3] = ppu_internal_ram;
            break;
        case NAMETABLE_MIRRORING_SINGLE_HIGH:
            emu.currentNtPtr[0] = ppu_internal_ram+0x400;
            emu.currentNtPtr[1] = ppu_internal_ram+0x400;
            emu.currentNtPtr[2] = ppu_internal_ram+0x400;
            emu.currentNtPtr[3] = ppu_internal_ram+0x400;
            break;
        case NAMETABLE_MIRRORING_VERTICAL:
            emu.currentNtPtr[0] = ppu_internal_ram;
            emu.currentNtPtr[1] = ppu_internal_ram+0x400;
            emu.currentNtPtr[2] = ppu_internal_ram;
            emu.currentNtPtr[3] = ppu_internal_ram+0x400;
            break;
        case NAMETABLE_MIRRORING_HORIZONTAL:
            emu.currentNtPtr[0] = ppu_internal_ram;
            emu.currentNtPtr[1] = ppu_internal_ram;
            emu.currentNtPtr[2] = ppu_internal_ram+0x400;
            emu.currentNtPtr[3] = ppu_internal_ram+0x400;
            break;
        default:
            show_error("Error", "Invalid nametable mirroring requested");
            break;
    }
} 




//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//mapper.c

u8 default_cartridge_read(u16 adr) {
    if(adr >= 0x8000 && adr <= 0xFFFF) { // PRG-ROM
        return memory[adr];
    } 
    return 0;
}

void default_cartridge_write(u16 adr, u8 val) { } //Cannot Write to ROM


/*

u8 shiftRegister;
u8 shiftCount;
u8 chrBankMode;
u8 prgBankMode;

void mapper001_write(u16 adr, u8 val)
{
    u8 reset = (val&0x80)!=0;
    if(reset)
    {
        //printf("reset\n");
        shiftRegister = 0;
        shiftCount = 0;
    }
    else
    {
        shiftRegister >>= 1;
        shiftRegister |= (val&1)<<4;
        shiftCount++;
        if(shiftCount == 5) // 5th write
        {
            u8 high3 = (adr>>12)&(~1);
            switch(high3)
            {
                case 0x8: // control
                    {
                        u8 mirroring = shiftRegister&0x3;
                        u8 prgMode = (shiftRegister>>2)&0x3;
                        switch(mirroring)
                        {
                            case 0: emu_set_nt_mirroring(NAMETABLE_MIRRORING_SINGLE_LOW); break;
                            case 1: emu_set_nt_mirroring(NAMETABLE_MIRRORING_SINGLE_HIGH); break;
                            case 2: emu_set_nt_mirroring(NAMETABLE_MIRRORING_VERTICAL); break;
                            case 3: emu_set_nt_mirroring(NAMETABLE_MIRRORING_HORIZONTAL); break;
                            default: break;
                        }
                        switch(prgMode)
                        {
                            case 0:
                            case 1:
                                prgBankMode = 0;
                                printf("prg bank mode: switch 32k\n");
                                break;
                            case 2:
                                prgBankMode = 2;
                                printf("prg bank mode: fix 0x8000\n");
                                break;
                            case 3:
                                prgBankMode = 3;
                                printf("prg bank mode: fix 0xC000\n");
                                break;
                            default: break;
                        }
                        if((shiftRegister&0x10) != 0) // chr bank 4k
                            chrBankMode = 1;
                        else // chr bank 8k
                            chrBankMode = 0;
                    }
                    break;
                case 0xA: // chr 0
                    //printf("CHR0 to %02d %02d\n", shiftRegister>>1, shiftRegister);
                    if(chrBankMode == 0)
                    {
                        ///assert((shiftRegister>>1) < emu.chrRomBlockCount);
                        ///emu.currentChrLowPtr = emu.chrRomBlocks[shiftRegister>>1];
                        ///emu.currentChrHiPtr = emu.chrRomBlocks[shiftRegister>>1]+4096;
                        emu.currentChrLowPtr = &ppu_cartridge_CHR[shiftRegister>>1];
                        emu.currentChrHiPtr = &ppu_cartridge_CHR[shiftRegister>>1]+4096;
                        
                    }
                    else
                    {
                        ///assert((shiftRegister>>1) < emu.chrRomBlockCount);
                        u32 add = (shiftRegister&0x1)?4096:0;
                        ///emu.currentChrLowPtr = emu.chrRomBlocks[shiftRegister>>1]+add;
                        emu.currentChrLowPtr = &ppu_cartridge_CHR[shiftRegister>>1]+add;
                    }
                    break;
                case 0xC: // chr 1
                    if(chrBankMode == 1)
                    {
                        //printf("CHR1 to %02d %02d\n", shiftRegister>>1, shiftRegister);
                        ///assert((shiftRegister>>1) < emu.chrRomBlockCount);
                        u32 add = (shiftRegister&0x1)?4096:0;
                        ///emu.currentChrHiPtr = emu.chrRomBlocks[shiftRegister>>1]+add;
                        emu.currentChrHiPtr = &ppu_cartridge_CHR[shiftRegister>>1]+add;
                    }
                    break;
                case 0xE: // prg
                    printf("PRG to %04d / %04d\n", shiftRegister, emu.prgRamBlockCount);
                    switch(prgBankMode)
                    {
                        case 0:
                            ///assert((shiftRegister&0xE)+1 < emu.prgRamBlockCount);
                            emu.currentPrg1Ptr = emu.prgRamBlocks[shiftRegister&0xE];
                            emu.currentPrg2Ptr = emu.prgRamBlocks[(shiftRegister&0xE)+1];
                            break;
                        case 2:
                            ///assert((shiftRegister&0xF) < emu.prgRamBlockCount);
                            emu.currentPrg1Ptr = emu.prgRamBlocks[0];
                            emu.currentPrg2Ptr = emu.prgRamBlocks[shiftRegister&0xF];
                            break;
                        case 3: 
                            /////assert((shiftRegister&0xF) < emu.prgRamBlockCount);
                            if((shiftRegister&0xF) >= emu.prgRamBlockCount)
                                break;
                            emu.currentPrg1Ptr = emu.prgRamBlocks[shiftRegister&0xF];
                            emu.currentPrg2Ptr = emu.prgRamBlocks[emu.prgRamBlockCount-1];
                            break;
                        default: break;
                    }
                    break;
            }
            //TODO: change bank
            shiftRegister = 0;
            shiftCount = 0;
        }
    }
}

u8 mapper003_read(u16 adr)
{
    return 0;
}

void mapper003_write(u16 adr, u8 val)
{
    if(val < emu.chrRomBlockCount)
    {
        ///emu.currentChrLowPtr = emu.chrRomBlocks[val];
        ///emu.currentChrHiPtr = emu.chrRomBlocks[val]+4096;
        emu.currentChrLowPtr = (unsigned char*)ppu_cartridge_CHR[val];
        emu.currentChrHiPtr = (unsigned char*)ppu_cartridge_CHR[val]+4096;

        
    }
    else
        ///assert(FALSE);
}

b32 mapper_init(u32 mapperId)
{
///
    ///emu.crRead = default_cartridge_read;
    ///emu.crWrite = default_cartridge_write;
    switch(mapperId)
    {
        case 0:
            break;
        case 1:
            if(emu.prgRamBlockCount > 16)
            {
                show_error("Error", "TODO: Highest CHR line switches 256KB PRG banks (1)");
                exit(-1);
            }
            
            ///emu.crWrite = mapper001_write;
            break;
        case 3:
            ///emu.crWrite = mapper003_write;
            break;
        default:
            return 0;
    }
    return 1;
} 
*/

char NESHEADER[16] = {0};
 
void nes_init(char *fileNameBuffer) {
   long _length;
   File f = SD.open(fileNameBuffer); 
   // Read NES Header 
   for (int i = 0; i < 16; i++) {
      if (f.available()) NESHEADER[i]=f.read();
   }
  
   tft.println("Reading ROM...");

   if (NESHEADER) {
      // Check if header matches "NES" + DOS EOF
      if (NESHEADER[0] != 'N' || NESHEADER[1] != 'E' || NESHEADER[2] != 'S' || NESHEADER[3] != 0x1A) {
         tft.println("Not NES ROM...");
      }

      u8 prgSize = NESHEADER[4]; // number of 16k units
      u8 chrSize = NESHEADER[5]; // number of 8k units
      u8 f6 = NESHEADER[6];
      u8 f7 = NESHEADER[7];
      u8 prgRAMSize = NESHEADER[8]; // 0 means 8k, number of 8k units
      u8 f9 = NESHEADER[9];
      u8 f10 = NESHEADER[10];
      b32 hasTrainer = (0x04&f6)!=0;
      b32 ntMirroring = f6&1; // 0 = horizontal, 1 = vertical
      u8 mapperNumber = (f7&0xF0) | (f6 >> 4);
      b32 isPal = (0x01&f9)!=0;

      emu.tvSystem = isPal ? TV_SYSTEM_PAL : TV_SYSTEM_NTSC;
      cpu.clockHz = isPal ? 1662607 : 1789773;
      apu.clocksPerFrame = cpu.clockHz/480; // 240Hz
      ///printf("clocks per frame %d\n", apu.clocksPerFrame);

      cpu.decimalFlag = 1; // set bit 5 to 1

      /*printf("PRG ROM %dB\n", prgSize*(1<<14));
      printf("PRG RAM %dB\n", prgRAMSize*(1<<13));
      printf("CHR ROM %dB (%d blocks)\n", chrSize*(1<<13), chrSize);
      printf("hastrainer: %d\nmapperNumber: %d \nTV:%s\n",hasTrainer, mapperNumber, isPal?"PAL":"NTSC");*/
//------------------------------------------------------------------------------
      for (int i = 0x8000; i < 0x10000; i++) { // READ 32kB PRGROM
         if (f.available()) memory[i]=f.read();
      }
      for (int i = 0; i < 8192; i++) {
         if (f.available()) ppu_cartridge_CHR[i]=f.read(); // READ 8kB CHROM
      }
//------------------------------------------------------------------------------
      emu.chrRomBlockCount = chrSize;
      emu_set_nt_mirroring(ntMirroring ? NAMETABLE_MIRRORING_VERTICAL : NAMETABLE_MIRRORING_HORIZONTAL);
      ///mapper_init(mapperNumber);
   }
   tft.println("Loaded.\n");
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//********************************************************************************
// MAIN SETUP
//********************************************************************************
void setup(){
  ///Serial.begin(9600);
//--------------------------------------------------------------------------------
  tft.begin();
  tft.setRotation(iliRotation270);  // landscape
  tft.setFont(Arial_bold_14);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextScale(1);
  tft.setTextLetterSpacing(2);
//--------------------------------------------------------------------------------
  tft.println("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    tft.println("SD card initialization failed!");
  } else {
    tft.println("SD card initialization done.");
  }  
//********************************************************************************   
  nes_init("smb.nes");

  
  tft.println("NES been loaded.");
    
    u8 pcLow = read_byte(0xFFFC);
    u8 pcHigh = read_byte(0xFFFD);
    cpu.PC = (pcHigh << 8) | pcLow;
    tft.print("PC set to " );
    tft.println(cpu.PC);
    cpu.S = 0;
    
    

}

void nes_frame() {  
   u32 i; 
   for(i = 0; ppu_cycle() == FALSE; i++) {
      if (i%3 == 0) { // cpu runs at 3x lower clock than ppu
         cpu_cycle();
         ///if((i%6) == 0) apu_cycle(); // apu runs every second cpu clock
      }
   }
} 

void loop() {
   nes_frame(); 
}
