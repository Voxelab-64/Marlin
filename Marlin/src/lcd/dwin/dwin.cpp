/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * DWIN by Creality3D
 */

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_CREALITY_LCD)

#include "dwin.h"

#include <WString.h>
#include <stdio.h>
#include <string.h>

#include "../fontutils.h"
#include "../ultralcd.h"

#include "../../sd/cardreader.h"

#include "../../MarlinCore.h"
#include "../../core/serial.h"
#include "../../core/macros.h"
#include "../../gcode/queue.h"

#include "../../feature/powerloss.h"
#include "../../feature/babystep.h"

#include "../../module/configuration_store.h"
#include "../../module/temperature.h"
#include "../../module/printcounter.h"
#include "../../module/motion.h"
#include "../../module/planner.h"

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#include "../../libs/buzzer.h"

#ifndef MACHINE_SIZE
  #define MACHINE_SIZE "220x220x250"
#endif
#ifndef CORP_WEBSITE_C
  #define CORP_WEBSITE_C "www.sz3dp.com"
#endif
#ifndef CORP_WEBSITE_E
  #define CORP_WEBSITE_E "www.sz3dp.com"
#endif

#define firmware_version  "v1.0.0"

#define PAUSE_HEAT true

//#define USE_STRING_HEADINGS

#define MENU_FONT   font8x16
#define STAT_FONT   font10x20
#define HEADER_FONT font10x20

#define MENU_CHAR_LIMIT  12

// Fan speed limit
#define FANON           255
#define FANOFF          0

// Print speed limit
#define MAX_PRINT_SPEED   999
#define MIN_PRINT_SPEED   10

// Temp limits
#if HAS_HOTEND
  #define MAX_E_TEMP    (HEATER_0_MAXTEMP - (HOTEND_OVERSHOOT))
  #define MIN_E_TEMP    HEATER_0_MINTEMP
#endif

#if HAS_HEATED_BED
  #define MIN_BED_TEMP  BED_MINTEMP
#endif

// Feedspeed limit (max feedspeed = DEFAULT_MAX_FEEDRATE * 2)
#define MIN_MAXFEEDSPEED      1
#define MIN_MAXACCELERATION   1
#define MIN_MAXCORNER         0.1
#define MIN_STEP              1

#define FEEDRATE_E      (60)

// Mininum unit (0.1) : multiple (10)
#define MINUNITMULT     10

#define ENCODER_WAIT    20
#define DWIN_SCROLL_UPDATE_INTERVAL 2000
#define DWIN_REMAIN_TIME_UPDATE_INTERVAL 20000

constexpr uint16_t TROWS = 5, MROWS = TROWS - 1,        // Total rows, and other-than-Back
                   TITLE_HEIGHT = 40,                   // Title bar height
                   MLINE = 44,                          // Menu line height
                   LBLX = 170,                           // Menu item label X
                   MENU_CHR_W = 4, STAT_CHR_W = 10;

#define MBASE(L) (55 + (L)*MLINE)
 
#define BABY_Z_VAR TERN(HAS_LEVELING, probe.offset.z, zprobe_zoffset)

/* Value Init */
HMI_value_t HMI_ValueStruct;
HMI_Flag HMI_flag{0};

millis_t Encoder_ms     = 0;
millis_t Wait_ms        = 0; 
millis_t dwin_heat_time = 0;

int checkkey = 0, last_checkkey = 0;

typedef struct {
  uint8_t now, last;
  void set(uint8_t v) { now = last = v; }
  void reset() { set(0); }
  bool changed() { bool c = (now != last); if (c) last = now; return c; }
  bool dec() { if (now) now--; return changed(); }
  bool inc(uint8_t v) { if (now < v) now++; else now = v; return changed(); }
} select_t;

select_t select_page{0}, select_file{0}, select_print{0}, select_prepare{0}
         , select_control{0}, select_axis{0}, select_temp{0}, select_motion{0}, select_tune{0}
         , select_PLA{0}, select_ABS{0}
         , select_speed{0}
         , select_acc{0}
         , select_corner{0}
         , select_step{0}
         // , select_leveling{0}
         ;

uint8_t index_file     = MROWS,
        index_prepare  = MROWS,
        index_control  = MROWS,
        index_leveling = MROWS,
        index_temperature = MROWS,
        index_tune     = MROWS,
        index_speed    = MROWS,
        index_acc      = MROWS,
        index_corner   = MROWS,
        index_step     =MROWS;
// char filebuf[50];
uint8_t countbuf = 0;

bool recovery_flag = false, abort_flag = false;

bool bltouch_flag = false,completedlevel_flag=false;

uint8_t filament_flag ;
bool filament_state = false;
bool filament_select = true;
bool filament_exist_select = false;
bool filament_recovery_flag=false;

constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

uint8_t Percentrecord = 0;
uint16_t last_Printtime = 0, remain_time = 0;
float last_temp_hotend_target = 0, last_temp_bed_target = 0;
float last_temp_hotend_current = 0, last_temp_bed_current = 0;
uint8_t last_fan_speed = 0;
uint16_t last_speed = 0;
float last_E_scale = 0;
bool DWIN_lcd_sd_status = 0;
bool pause_action_flag = 0;
int temphot = 0, tempbed = 0;
float zprobe_zoffset = 0;
float last_zoffset = 0, last_probe_zoffset = 0;

#define DWIN_LANGUAGE_EEPROM_ADDRESS 0x01   // Between 0x01 and 0x63 (EEPROM_OFFSET-1)
                                            // BL24CXX::check() uses 0x00
#define FILAMENT_FLAG_EEPROM_ADDRESS 0x63

void filament_flag_select(void){
    BL24CXX::read(FILAMENT_FLAG_EEPROM_ADDRESS,(uint8_t*)&filament_flag,sizeof(filament_flag));
}
void set_no_filament(void){
      filament_flag=0;
      BL24CXX::write(FILAMENT_FLAG_EEPROM_ADDRESS,(uint8_t*)&filament_flag,sizeof(filament_flag));
}
void set_filament(void){
      filament_flag=1;
      BL24CXX::write(FILAMENT_FLAG_EEPROM_ADDRESS,(uint8_t*)&filament_flag,sizeof(filament_flag));
}

void lcd_select_language(void) {
  BL24CXX::read(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language_flag, sizeof(HMI_flag.language_flag));
  if (HMI_flag.language_flag)
    DWIN_JPG_CacheTo1(Language_Chinese);
  else
    DWIN_JPG_CacheTo1(Language_English);
}

void set_english_to_eeprom(void) {
  HMI_flag.language_flag = 0;
  DWIN_JPG_CacheTo1(Language_English);
  BL24CXX::write(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language_flag, sizeof(HMI_flag.language_flag));
}
void set_chinese_to_eeprom(void) {
  HMI_flag.language_flag = 1;
  DWIN_JPG_CacheTo1(Language_Chinese);
  BL24CXX::write(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&HMI_flag.language_flag, sizeof(HMI_flag.language_flag));
}

// void show_plus_or_minus(uint8_t size, uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value) {
//   if (value < 0) {
//     DWIN_Draw_String(false, true, size, White, bColor, x - 6, y, (char*)"-");
//     DWIN_Draw_FloatValue(true, true, 0, size, White, bColor, iNum, fNum, x, y, -value);
//   }
//   else {
//     DWIN_Draw_String(false, true, size, White, bColor, x - 6, y, (char*)" ");
//     DWIN_Draw_FloatValue(true, true, 0, size, White, bColor, iNum, fNum, x, y, value);
//   }
// }



void show_plus_or_minus(uint8_t size, uint16_t Color, uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, long value) {
    if (value < 0) {
      DWIN_Draw_String(false, true, size, Color, bColor, x - 6, y, (char*)"-");
      DWIN_Draw_FloatValue(true, true, 0, size, Color, bColor, iNum, fNum, x, y, -value);
    }
    else {
      DWIN_Draw_String(false, true, size, Color, bColor, x - 6, y, (char*)" ");
      DWIN_Draw_FloatValue(true, true, 0, size, Color, bColor, iNum, fNum, x, y, value);
    }
}
void ICON_Print() {
  //  DWIN_JPG_CacheTo1(Language_Chinese);
  if (select_page.now == 0) {
    //DWIN_ICON_Show(ICON, 2, 154, 60);
    //DWIN_ICON_Show(ICON2,1,154, 60);
    if (HMI_flag.language_flag)
      //DWIN_Frame_AreaCopy(1, 0, 20, 30, 32, 212, 126);
       DWIN_ICON_Show(ICON2,2,154, 60);
    else
     // DWIN_Frame_AreaCopy(1, 0, 20, 30, 32, 212, 126);
      DWIN_ICON_Show(ICON2,4,154, 60);
  }
  else {
    //DWIN_ICON_Show(ICON, 1, 154, 60);
    if (HMI_flag.language_flag)
     DWIN_ICON_Show(ICON2,1,154, 60);
      //DWIN_Frame_AreaCopy(1, 0, 2, 30, 16, 212, 126);
    else
     DWIN_ICON_Show(ICON2,3,154, 60);
      // DWIN_Frame_AreaCopy(1, 0, 2, 30, 16, 212, 126);
  }
}

void ICON_Control() {
  if (select_page.now == 1) {
     // DWIN_ICON_Show(ICON, 4, 306, 60);
    if (HMI_flag.language_flag)
      DWIN_ICON_Show(ICON2, 6, 306, 60);
    // DWIN_Frame_AreaCopy(1, 32, 20, 62, 32, 360, 126);
    else
     DWIN_ICON_Show(ICON2, 8, 306, 60);
    // DWIN_Frame_AreaCopy(1, 32, 20, 84, 32, 350, 126);
  }
  else {
     // DWIN_ICON_Show(ICON, 3, 306, 60);
    if (HMI_flag.language_flag)
     DWIN_ICON_Show(ICON2, 5, 306, 60);
    // DWIN_Frame_AreaCopy(1, 32, 2, 62, 16, 360, 126);
    else
     DWIN_ICON_Show(ICON2, 7, 306, 60);
     // DWIN_Frame_AreaCopy(1,32, 2, 84, 16, 350, 126);
  }

}


void ICON_Prepare() {
  if (select_page.now == 2) {
    //DWIN_ICON_Show(ICON, 6, 154, 162);
    if (HMI_flag.language_flag)
     DWIN_ICON_Show(ICON2, 10, 154, 162);
     // DWIN_Frame_AreaCopy(1, 64, 20, 94, 32, 212, 228);
    else
     DWIN_ICON_Show(ICON2, 12, 154, 162);
     // DWIN_Frame_AreaCopy(1, 86, 20, 135, 34, 202, 228);
  }
  else {
   // DWIN_ICON_Show(ICON, 5, 154, 162);
    if (HMI_flag.language_flag)
     DWIN_ICON_Show(ICON2, 9, 154, 162);
     //   DWIN_Frame_AreaCopy(1, 64, 2, 94, 16, 212, 228);
    else
     DWIN_ICON_Show(ICON2, 11, 154, 162);
      //  DWIN_Frame_AreaCopy(1, 86, 2, 135, 16, 202, 228);
  }
}

void ICON_StartInfo(bool show) {
  if (show) {
   // DWIN_ICON_Show(ICON, 8, 306, 162);
    if (HMI_flag.language_flag) 
     DWIN_ICON_Show(ICON2, 14, 306, 162);
    //DWIN_Frame_AreaCopy(1,96, 20, 124, 32, 360, 228);
    else
      DWIN_ICON_Show(ICON2, 16, 306, 162);
    // DWIN_Frame_AreaCopy(1,135, 20, 165, 34, 360, 228);
  }
  else {
   // DWIN_ICON_Show(ICON, 7, 306, 162);
    if (HMI_flag.language_flag)
      DWIN_ICON_Show(ICON2, 13, 306, 162);
     // DWIN_Frame_AreaCopy(1,96, 2, 124, 16, 360, 228);
    else
      DWIN_ICON_Show(ICON2, 15, 306, 162);
     // DWIN_Frame_AreaCopy(1,135, 2, 165, 16, 360, 228);
  }
}



void ICON_Leveling(bool show) {
  if (show) {
    DWIN_ICON_Show(ICON, ICON_Leveling_1, 145, 246);
    if (HMI_flag.language_flag)
      DWIN_Frame_AreaCopy(1, 211, 447, 238, 479 - 19, 186, 318);
    else
      DWIN_Frame_AreaCopy(1, 84, 437, 120,  449, 200 - 18, 318);
  }
  else {
    DWIN_ICON_Show(ICON, ICON_Leveling_0, 145, 246);
    if (HMI_flag.language_flag)
      DWIN_Frame_AreaCopy(1, 211, 405, 238, 420, 186, 318);
    else
      DWIN_Frame_AreaCopy(1, 84, 465, 120, 478, 200 - 18, 318);
  }
}

void ICON_Tune() {
  if (select_print.now == 0) {
     DWIN_ICON_Show(ICON,20,390,182);
  }
  else {
    DWIN_ICON_Show(ICON,19,390,182);
  }
}

void ICON_Pause() {
  if (select_print.now == 1) {
      DWIN_ICON_Show(ICON,18,310,182);
  }
  else {
     DWIN_ICON_Show(ICON,17,310,182);
  }
}

void ICON_Continue() {
  if (select_print.now == 1) {
     DWIN_ICON_Show(ICON,16,310,182);
  }
  else {
     DWIN_ICON_Show(ICON,15,310,182);
  }
}

void ICON_Stop() {
  if (select_print.now == 2) {
     DWIN_ICON_Show(ICON,14,230,182);
  }
  else {
     DWIN_ICON_Show(ICON,13,230,182);
  }
}

inline void Draw_Steady_State(void)
{
  DWIN_Draw_Line(0X0000,140,40,450,40);
  DWIN_ICON_Show(ICON, ICON_LOGO, 20, 56);

  DWIN_ICON_Show(ICON, 9, 20, 194);
  #if HOTENDS > 1
    // DWIN_ICON_Show(ICON,ICON_HotendTemp, 13, 381);
  #endif
  DWIN_ICON_Show(ICON, 10,   20, 230);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 50, 200, thermalManager.temp_hotend[0].celsius);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 50 + 4 * STAT_CHR_W + 6, 200, thermalManager.temp_hotend[0].target);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 50, 236, thermalManager.temp_bed.celsius);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 50 + 4 * STAT_CHR_W + 6, 236, thermalManager.temp_bed.target);
  DWIN_Draw_String(false, false, font8x16, Black, White, 50 + 3 * STAT_CHR_W + 5, 200, (char*)"/");
  DWIN_Draw_String(false, false, font8x16, Black, White, 50 + 3 * STAT_CHR_W + 5, 236, (char*)"/");
  DWIN_UpdateLCD();
}

inline void Clear_Title_Bar(void) {
 // DWIN_Draw_Rectangle(1, Background_blue,   0,  0,  DWIN_WIDTH,  30);
}

inline void Draw_Title(const char * const title) {
  DWIN_Draw_String(false, false, HEADER_FONT, 0X0000, 0XFFFF, 230, 8, (char*)title);
}

inline void Clear_Menu_Area(void) {
  DWIN_Draw_Rectangle(1, Background_black,  0,  0, 480,  272);
}



inline void Clear_Main_Window(void) {
  //Clear_Title_Bar();
  Clear_Menu_Area();
  DWIN_UpdateLCD();
}

inline void Clear_Popup_Area(void) {
  Clear_Title_Bar();
  DWIN_Draw_Rectangle(1, Background_black,  0,  0, 480,  272);
}

void Draw_Popup_Bkgd_105(void) {
  DWIN_Draw_Rectangle(1, White, 100, 35, 380, 235);
}

// inline void Draw_More_Icon(const uint8_t line) {
//   DWIN_ICON_Show(ICON, ICON_More, 420, 54 + line * MLINE);
// }

inline void Draw_More_Right_Icon(const uint8_t line){
  DWIN_ICON_Show(ICON,26, 420, 54 + line * MLINE);
}

inline void Draw_Menu_Cursor(const uint8_t line) {
  // DWIN_ICON_Show(ICON,ICON_Rectangle, 0, 31 + line * MLINE);
  DWIN_Draw_Rectangle(1, Red_Color, 140, 40 + line * MLINE, 150, 40 + (line + 1) * MLINE - 2);
}

inline void Erase_Menu_Cursor(const uint8_t line) {
  DWIN_Draw_Rectangle(1, Background_black, 140, 40 + line * MLINE, 150, 40 + (line + 1) * MLINE - 2);
}

inline void Move_Highlight(const int16_t from, const uint16_t newline) {
  Erase_Menu_Cursor(newline - from);
  Draw_Menu_Cursor(newline);
}

inline void Add_Menu_Line() {
  Move_Highlight(1, MROWS);
  DWIN_Draw_Line(Line_Color, 160, 40 + MROWS * MLINE, 450, 40 + MROWS * MLINE);
}

inline void Scroll_Menu(const uint8_t dir) {
  DWIN_Frame_AreaMove(1, dir, MLINE, Background_black, 160, 45, 470, 260);
  switch (dir) {
    case DWIN_SCROLL_DOWN: Move_Highlight(-1, 0); break;
    case DWIN_SCROLL_UP:   Add_Menu_Line(); break;
  }
}

inline uint16_t nr_sd_menu_items() {
  return card.get_num_Files() + !card.flag.workDirIsRoot;
}

inline void Draw_Menu_Icon(const uint8_t line, const uint8_t icon) {
  DWIN_ICON_Show(ICON, icon, 26, 46 + line * MLINE);
}

inline void Erase_Menu_Text(const uint8_t line) {
  DWIN_Draw_Rectangle(1, Background_black, LBLX, 31 + line * MLINE + 4, 271, 28 + (line + 1) * MLINE - 4);
}

inline void Draw_Menu_Line(const uint8_t line, const uint8_t icon=0, const char * const label=nullptr) {
  if (label) DWIN_Draw_String(false, false, font8x16, Black, Background_black, LBLX, 52 + line * MLINE, (char*)label);
  //if (icon) Draw_Menu_Icon(line, icon);
  DWIN_Draw_Line(Line_Color, 160, 40 + (line + 1) * MLINE, 450, 40 + (line + 1) * MLINE);
}

// The "Back" label is always on the first line
inline void Draw_Back_Label(void) {
  if (HMI_flag.language_flag)
  {
      DWIN_Frame_AreaCopy(1, 124, 50, 150,64, LBLX, MBASE(0));
      DWIN_ICON_Show(ICON,25,LBLX-10,MBASE(0));
  }
  else
  {
      DWIN_Frame_AreaCopy(1, 340, 166, 400,180, LBLX, MBASE(0));
      DWIN_ICON_Show(ICON,25,LBLX-10,MBASE(0));
  }
}

// Draw "Back" line at the top
inline void Draw_Back_First(const bool is_sel=true) {
  Draw_Menu_Line(0, ICON_Back);
  Draw_Back_Label();
  if (is_sel) Draw_Menu_Cursor(0);
}

//
// Draw Menus
//

inline void draw_move_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 360, 50, 420, 64, LBLX, line); // "Move"
}

inline void Prepare_Item_Move(const uint8_t row) {
  if (HMI_flag.language_flag)
      DWIN_Frame_AreaCopy(1,156,50,200,66,LBLX, MBASE(row));
  else
    draw_move_en(MBASE(row)); // "Move >"
  Draw_Menu_Line(row, ICON_Axis);
  Draw_More_Right_Icon(row);
}

inline void Prepare_Item_Disable(const uint8_t row) {
  if (HMI_flag.language_flag)
      DWIN_Frame_AreaCopy(1,202,50,262,66,LBLX, MBASE(row));
  else
      DWIN_Frame_AreaCopy(1,156,68,275,82,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_CloseMotor);
}
inline void Prepare_Item_Bltouch(const uint8_t row){
  if(HMI_flag.language_flag)
    //DWIN_Frame_AreaCopy(3,0,2,48,20,LBLX, MBASE(row));
    DWIN_ICON_Show(ICON1,28,LBLX,MBASE(row));
  else
    //DWIN_Frame_AreaCopy(3,0,2,48,20,LBLX, MBASE(row));
  DWIN_ICON_Show(ICON1,29,LBLX,MBASE(row));
    Draw_Menu_Line(row, ICON_CloseMotor);
}
inline void Prepare_Item_Home(const uint8_t row) {
  if (HMI_flag.language_flag)
  DWIN_Frame_AreaCopy(1,264,50,310,66,LBLX, MBASE(row));
  else
    DWIN_Frame_AreaCopy(1,278,68,400,82,LBLX, MBASE(row));
  Draw_Menu_Line(row, ICON_Homing);
}

inline void Prepare_Item_Offset(const uint8_t row) {
  if (HMI_flag.language_flag) {
    #if 0
      DWIN_Frame_AreaCopy(1, 174, 164, 271 - 48, 479 - 302, LBLX, MBASE(row));
      show_plus_or_minus(font8x16, Background_black, 2, 2, 202, MBASE(row), probe.offset.z * 100);
    #else
     // DWIN_Frame_AreaCopy(1, 43, 89, 271 - 173, 479 - 378, LBLX, MBASE(row));
        DWIN_Frame_AreaCopy(1, 312, 50, 372, 66, LBLX, MBASE(row));
    #endif
  }
  else {
    #if 0
      DWIN_Frame_AreaCopy(1, 93, 179, 271 - 130, 479 - 290, LBLX, MBASE(row)); // "Z-Offset"
      show_plus_or_minus(font8x16, Background_black, 2, 2, 202, MBASE(row), probe.offset.z * 100);
    #else
        DWIN_Frame_AreaCopy(1,0,84,125,100,LBLX, MBASE(row));
    #endif
  }
  Draw_Menu_Line(row, ICON_SetHome);
}

inline void Prepare_Item_PLA(const uint8_t row) {
  if (HMI_flag.language_flag) {
     DWIN_Frame_AreaCopy(1,374,50,428,66,LBLX, MBASE(row));
  }
  else {
    DWIN_Frame_AreaCopy(1, 420, 18, 474, 34, LBLX, MBASE(row)); // "Preheat"
    DWIN_Frame_AreaCopy(1, 40, 166, 65, 180, LBLX + 49 + 12, MBASE(row)); // "PLA"
   
  }
  Draw_Menu_Line(row, ICON_PLAPreheat);
}

inline void Prepare_Item_ABS(const uint8_t row) {
  if (HMI_flag.language_flag) {
     DWIN_Frame_AreaCopy(1,0,68,55,84,LBLX, MBASE(row));
  }
  else {
    DWIN_Frame_AreaCopy(1, 420, 18, 474, 34, LBLX, MBASE(row)); // "Preheat"
    DWIN_Frame_AreaCopy(1, 28, 48, 55, 64, LBLX + 49 + 12, MBASE(row)); // "ABS"
  
  }
  Draw_Menu_Line(row, ICON_ABSPreheat);
}

inline void Prepare_Item_Cool(const uint8_t row) {
  if (HMI_flag.language_flag)
    //DWIN_Frame_AreaCopy(1, 1, 104, 271 - 215, 479 - 362, LBLX, MBASE(row));
      DWIN_Frame_AreaCopy(1,90,68,148,84,LBLX, MBASE(row));
  else
     DWIN_Frame_AreaCopy(1, 235, 86, 310, 102, LBLX, MBASE(row)); // "Cooldown" 
  Draw_Menu_Line(row, ICON_Cool);
}

inline void Prepare_Item_feeding(const uint8_t row) {
    if (HMI_flag.language_flag)
      DWIN_Frame_AreaCopy(1,220,122,280,138,LBLX, MBASE(row));
  else
  // "Cooldown"
      DWIN_Frame_AreaCopy(1,0,205,115,220,LBLX, MBASE(row));
      Draw_Menu_Line(row, ICON_Cool);
}

inline void Prepare_Item_Desizing (const uint8_t row) {
    if (HMI_flag.language_flag)
      DWIN_Frame_AreaCopy(1,282,122,340,138,LBLX, MBASE(row));
  else
      DWIN_Frame_AreaCopy(1,120,205,255,220,LBLX, MBASE(row));
      Draw_Menu_Line(row, ICON_Cool);
}

inline void Prepare_Item_Lang(const uint8_t row) {
  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1,370,20,430,36,LBLX, MBASE(row));
    DWIN_Draw_String(false, false, font8x16, Black, Background_black, 410, MBASE(row), (char*)"CN");
  }
  else {
    DWIN_Frame_AreaCopy(1, 0, 183, 145, 200, LBLX, MBASE(row)); // "Language selection"
    DWIN_Draw_String(false, false, font8x16, Black, Background_black, 410, MBASE(row), (char*)"EN");
  }
}

inline void Draw_Zoffset_Menu(const uint8_t row)
{
      if(HMI_flag.language_flag)
      {
            DWIN_Frame_AreaCopy(1,60,122,120,138,LBLX,MBASE(row));
      }
      else
      {
          DWIN_Frame_AreaCopy(1, 275, 166, 330, 180, LBLX, MBASE(row));
      }
       DWIN_Draw_Line(Line_Color, 160, 40 + (row + 1) * MLINE, 450, 40 + (row + 1) * MLINE);
}

inline void Draw_feeding_Menu(const uint8_t row)
{
      if(HMI_flag.language_flag)
      {
            DWIN_Frame_AreaCopy(1,430,68,475,84,LBLX,MBASE(row));
      }
      else
      {
              DWIN_Frame_AreaCopy(1,300,202,400,222,LBLX,MBASE(row));
      }
}

inline void Draw_Extruder_Menu(const uint8_t row)
{
    if(HMI_flag.language_flag){
        DWIN_Frame_AreaCopy(1, 220, 86, 278, 102, LBLX, MBASE(row));             
    }
    else
    {
          DWIN_Frame_AreaCopy(1,155,185,210,201,LBLX,MBASE(row));
          
    }
    DWIN_Draw_Line(Line_Color, 160, 40 + (row + 1) * MLINE, 450, 40 + (row + 1) * MLINE);
}

inline void Draw_ABStemp_Menu(const uint8_t row)
{
    if(HMI_flag.language_flag){
        DWIN_Frame_AreaCopy(1,220,20,306,36,LBLX, MBASE(row));              
    }
    else
    {
        DWIN_Frame_AreaCopy(1, 420, 18, 474, 34, LBLX, MBASE(row)); // "Preheat"
        DWIN_Frame_AreaCopy(1, 28, 48, 55, 64, LBLX + 49 + 12, MBASE(row)); // "ABS"
        DWIN_Frame_AreaCopy(1,220,48,270,64,LBLX +30 +49 + 12, MBASE(row));
         
    }
    DWIN_Draw_Line(Line_Color, 160, 40 + (row + 1) * MLINE, 450, 40 + (row + 1) * MLINE);
    Draw_More_Right_Icon(row);
}

inline void Draw_Resume_Menu(const uint8_t row)
{
        if(HMI_flag.language_flag){
               DWIN_Frame_AreaCopy(1,370,68,430,84,LBLX, MBASE(row)); 
        }
        else{
                DWIN_Frame_AreaCopy(1,220,100,260,120,LBLX, MBASE(row));
                DWIN_Frame_AreaCopy(1,42,100,140,120,LBLX+45, MBASE(row));
        }
     DWIN_Draw_Line(Line_Color, 160, 40 + (row + 1) * MLINE, 450, 40 + (row + 1) * MLINE);
}

inline void Draw_temp_Menu(const uint8_t row)
{
  if(HMI_flag.language_flag)
  {
     DWIN_Frame_AreaCopy(1,250,86,278,102, LBLX,MBASE(row));
  }
  else
  {
      DWIN_Frame_AreaCopy(1,235,20,330,40, LBLX,MBASE(row));//temperature
  }
     DWIN_Draw_Line(Line_Color, 160, 40 + (row + 1) * MLINE, 450, 40 + (row + 1) * MLINE);
     Draw_More_Right_Icon(row);
}

inline void Draw_Control_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
  const int16_t scroll = MROWS - index_control; // Scrolled-up lines
  #define PSCROL(L) (scroll + (L))
  #define PVISI(L)  WITHIN(PSCROL(L), 0, MROWS)
  if (HMI_flag.language_flag) {
     DWIN_Frame_AreaCopy(1,240,2,270,16,285,12); //控制
  }
  else
  {
      DWIN_Frame_AreaCopy(1,100, 250, 158, 270, 270, 12);
  }
  if (PVISI(0)) Draw_Back_First(select_control.now == 0); // < Back
  if (PVISI(1)) 
    {
      Prepare_Item_Move(PSCROL(1));       // Move >
    }
  if (PVISI(2)) Prepare_Item_Disable(PSCROL(2));    // Disable Stepper
  if (PVISI(3)) Prepare_Item_Home(PSCROL(3));       // Auto Home
  if (PVISI(4)) Prepare_Item_Offset(PSCROL(4));     // Z-Offset
  if (PVISI(5)) Prepare_Item_PLA(PSCROL(5));        // Preheat PLA
  if (PVISI(6)) Prepare_Item_ABS(PSCROL(6));        // Preheat ABS
  if (PVISI(7)) Prepare_Item_feeding(PSCROL(7));
  if (PVISI(8)) Prepare_Item_Desizing(PSCROL(8));  
  if (PVISI(9)) Prepare_Item_Cool(PSCROL(9)); 
  
  if (select_control.now) Draw_Menu_Cursor(PSCROL(select_control.now));
}

inline void Draw_Prepare_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
  const int16_t scroll = TERN(0, MROWS - index_prepare, 0); // Scrolled-up lines

  #define CSCROL(L) (scroll + (L))
  #define CLINE(L)  MBASE(CSCROL(L))
  #define CVISI(L)  WITHIN(CSCROL(L), 0, MROWS)
  

  if (CVISI(0)) Draw_Back_First(select_prepare.now == 0);                        // < Back

  if (HMI_flag.language_flag) {
     DWIN_Frame_AreaCopy(1,208,2,238,16,285,12); //设置
                 
  if (CVISI(1)) 
  {
    DWIN_Frame_AreaCopy(1,250,86,278,102,LBLX, CLINE(1));// Temperature >
  }
  if (CVISI(2)) 
  {
      DWIN_Frame_AreaCopy(1,410,2,440,16, LBLX, CLINE(2));  // Motion >
  }
  if(CVISI(3))
  {
       DWIN_ICON_Show(ICON2,35,LBLX,CLINE(3));
      if(filament_flag==false)
      {
        DWIN_ICON_Show(ICON2,17,400,CLINE(3));
      }
      else
      {
          DWIN_ICON_Show(ICON2,18,400,CLINE(3));
      }
  }
  if (CVISI(4)) DWIN_Frame_AreaCopy(1,276,68,336,84,LBLX, CLINE(4)); // Read Config
  if (CVISI(5)) Prepare_Item_Lang(5);//DWIN_Frame_AreaCopy(1,62,122,120,138,LBLX, CLINE(4));   // 
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Setting"); // TODO: GET_TEXT_F
    #else
      DWIN_Frame_AreaCopy(1, 220, 50,270, 68, 270, 12);
    #endif
   if (CVISI(1))  DWIN_Frame_AreaCopy(1,235,20,330,40, LBLX,CLINE(1));//temperature
   if (CVISI(2)) DWIN_Frame_AreaCopy(1,330,20,385,40, LBLX, CLINE(2));  // Motion >
   if (CVISI(3))
   {
      //DWIN_ICON_Show(ICON2,36,LBLX,CLINE(3));
        DWIN_Frame_AreaCopy(1,0,20,142,38,LBLX,CLINE(3));
      if(filament_flag==false)
      {
          DWIN_ICON_Show(ICON2,17,400,CLINE(3));
      }
      else
      {
          DWIN_ICON_Show(ICON2,18,400,CLINE(3));
      }
   }
   if (CVISI(4))  DWIN_Frame_AreaCopy(1,180,100,220,120,LBLX, CLINE(4));
   if (CVISI(4))  DWIN_Frame_AreaCopy(1,40,100,140,120,LBLX+45, CLINE(4));// Read Config
   if (CVISI(5)) Prepare_Item_Lang(5);//DWIN_Frame_AreaCopy(1, 275, 166, 330, 180, LBLX, CLINE(4)); //zoffset
  }

  if (select_prepare.now && CVISI(select_prepare.now))
    Draw_Menu_Cursor(CSCROL(select_prepare.now));

  // Draw icons and lines
  LOOP_L_N(i, 5)
  if (CVISI(i + 1)) Draw_Menu_Line(CSCROL(i + 1), ICON_Temperature + i);

  Draw_More_Right_Icon(1);
  Draw_More_Right_Icon(2);

}

inline void Draw_Tune_Menu() {

  Clear_Main_Window();
  Draw_Steady_State();

  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 160,2,238,18 ,250, 12);

     DWIN_Frame_AreaCopy(1, 0, 122, 60, 138, LBLX, MBASE(1));
     DWIN_Frame_AreaCopy(1, 220, 86, 280, 102, LBLX, MBASE(2));
     DWIN_Frame_AreaCopy(1, 282, 86, 338, 102, LBLX, MBASE(3));
     DWIN_Frame_AreaCopy(1, 342, 86, 400, 102, LBLX, MBASE(4));
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Tune"); // TODO: GET_TEXT
    #else
      DWIN_Frame_AreaCopy(1, 280, 2, 312, 20, 275, 12);
    #endif
    //DWIN_Frame_AreaCopy(1, 1, 179, 271 - 179, 479 - 287 - 2, LBLX, MBASE(1)); // print speed
    DWIN_Frame_AreaCopy(1,0,64,56,82,LBLX, MBASE(1));
    DWIN_Frame_AreaCopy(1,30,118,72,136,LBLX+60, MBASE(1));
    DWIN_Frame_AreaCopy(1,300,102,348,118,LBLX, MBASE(2));
    DWIN_Frame_AreaCopy(1,312,84,404,100,LBLX+52,MBASE(2));
    DWIN_Frame_AreaCopy(1,350,102,376,118,LBLX, MBASE(3));
    DWIN_Frame_AreaCopy(1,312,84,404,100,LBLX+30,MBASE(3));
    DWIN_Frame_AreaCopy(1,0,120,76,140,LBLX, MBASE(4));
  }

  Draw_Back_First(select_tune.now == 0);
  if (select_tune.now) Draw_Menu_Cursor(select_tune.now);

  Draw_Menu_Line(1, ICON_Speed);
  Draw_Menu_Line(2, ICON_HotendTemp);
  Draw_Menu_Line(3, ICON_BedTemp);
  Draw_Menu_Line(4, ICON_FanSpeed);
  //Draw_Menu_Line(5, ICON_Zoffset);

  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(1+ MROWS - index_tune), feedrate_percentage);
  DWIN_Draw_String(false, false, MENU_FONT, Black, Background_black, 400+25, MBASE(1+ MROWS - index_tune),(char*)"%");
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(2+ MROWS - index_tune), thermalManager.temp_hotend[0].target);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(3+ MROWS - index_tune), thermalManager.temp_bed.target);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(4+ MROWS - index_tune), thermalManager.fan_speed[0]);
  show_plus_or_minus(font8x16, Black,Background_black, 2, 2, 400, MBASE(5 + MROWS - index_tune), BABY_Z_VAR * 100);
}

inline void draw_max_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 270, 120, 300, 140, LBLX, line);   // "Max"
}
inline void draw_max_accel_en(const uint16_t line) {
  draw_max_en(line);
  DWIN_Frame_AreaCopy(1, 305, 120, 410, 140, LBLX + 30 + 5, line); // "Acceleration"
}
inline void draw_speed_en(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 200, 120, 244, 140, LBLX + inset, line); // "Speed"
}
inline void draw_corner_en(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 80, 120, 128, 140, LBLX + 30 + 5, line); // "Corner"
}
inline void draw_steps_per_mm(const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 0, 145, 105, 165, LBLX, line);   // "Steps-per-mm"
}
inline void say_x(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 246, 120, 256, 140, LBLX + inset, line); // "X"
}
inline void say_y(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 265, 102, 275, 120, LBLX + inset, line); // "Y"
}
inline void say_z(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 275, 102, 285, 120, LBLX + inset, line); // "Z"
}
inline void say_e(const uint16_t inset, const uint16_t line) {
  DWIN_Frame_AreaCopy(1, 287, 102, 295, 120, LBLX + inset, line); // "E"
}

inline void Draw_Motion_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 410, 2, 439, 18, 285, 12);

    DWIN_Frame_AreaCopy(1, 406, 86, 460, 102, LBLX, MBASE(1));     // max speed
    DWIN_Frame_AreaCopy(1, 406, 86, 436, 102, LBLX, MBASE(2));     // max...
    DWIN_Frame_AreaCopy(1, 62, 104, 110, 120, LBLX + 27, MBASE(2)); // ...acceleration
    DWIN_Frame_AreaCopy(1, 406, 86, 432, 102, LBLX, MBASE(3));;     // max...
    DWIN_Frame_AreaCopy(1, 154, 122, 185, 138, LBLX + 27, MBASE(3)); // ...   
    DWIN_Frame_AreaCopy(1, 436, 86, 462, 102, LBLX + 60, MBASE(3)); // ...jerk
    DWIN_Frame_AreaCopy(1, 200, 104, 246, 120, LBLX, MBASE(4)); // flow ratio
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Motion"); // TODO: GET_TEXT_F
    #else
      DWIN_Frame_AreaCopy(1, 335, 20,387, 38, 275, 12);
    #endif
     draw_max_en(MBASE(1)); draw_speed_en(30 + 5, MBASE(1));               // "Max Speed"
     draw_max_accel_en(MBASE(2));                                          // "Max Acceleration"
     draw_max_en(MBASE(3)); draw_corner_en(MBASE(3));                      // "Max Corner"
     draw_steps_per_mm(MBASE(4));                                          // "Steps-per-mm"
  }

  Draw_Back_First(select_motion.now == 0);
  if (select_motion.now) Draw_Menu_Cursor(select_motion.now);

  LOOP_L_N(i, 4) Draw_Menu_Line(i + 1, ICON_MaxSpeed + i);
  Draw_More_Right_Icon(1);
  Draw_More_Right_Icon(2);
  Draw_More_Right_Icon(3);
  Draw_More_Right_Icon(4);
}

//
// Draw Popup Windows
//

void Popup_Window_Temperature(const bool toohigh) {
  Clear_Popup_Area();
  if (toohigh) {
      DWIN_Draw_String(false, true, font8x16, Black, Background_black, 160, 100, (char*)"Nozzle or Bed temperature");
      DWIN_Draw_String(false, true, font8x16, Black, Background_black, 180, 120, (char*)"is too high");
    
  }
  else {
      DWIN_Draw_String(false, true, font8x16, Black, Background_black, 160, 100, (char*)"Nozzle or Bed temperature");
      DWIN_Draw_String(false, true, font8x16, Black, Background_black, 180, 120, (char*)"is too low");
  }
}

inline void Draw_Popup_Bkgd_60() {
  DWIN_Draw_Rectangle(1, Background_window, 120, 36, 360, 236);
}

#if HAS_HOTEND

  void Popup_Window_ETempTooLow(void) {
    Clear_Main_Window();
    if (HMI_flag.language_flag) {
        DWIN_ICON_Show(ICON1,7,0,0);
    }
    else {
        DWIN_ICON_Show(ICON2,37,0,0);
    }
  }

#endif

void Popup_Window_Resume(void) {
  Clear_Popup_Area();
  if (HMI_flag.language_flag) {
      DWIN_ICON_Show(ICON,54,0,0);
      DWIN_ICON_Show(ICON,44,130,172);
      DWIN_ICON_Show(ICON,42,290,172);
  }
  else {
      DWIN_ICON_Show(ICON,55,0,0);
      DWIN_ICON_Show(ICON,49,130,172);
      DWIN_ICON_Show(ICON,46,290,172);
  }
}

void Popup_Window_Print_Home(void)
{
    DWIN_ICON_Show(ICON,28,0,0);
}

void Popup_Window_Home(void) {
  if(HMI_flag.language_flag)
  DWIN_ICON_Show(ICON, 30, 0, 0);
  else
  {
     DWIN_ICON_Show(ICON,52,0,0);
  }
  
}
void Popup_Window_Leveling(void) {
  Clear_Main_Window();
  Draw_Popup_Bkgd_60();
  DWIN_ICON_Show(ICON, ICON_AutoLeveling, 101, 105);
  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 0, 371, 100, 386, 84, 240);
    DWIN_Frame_AreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
    DWIN_Draw_String(false, true, font8x16,  Black, Background_black, 76, 230, (char*)"Auto leveling...");
    DWIN_Draw_String(false, true, font8x16,  Black, Background_black, 24, 260, (char*)"Please wait until completed");
  }
}

void Draw_Select_Highlight(const bool sel) {
  HMI_flag.select_flag = sel;
  // const uint16_t c1 = sel ? Select_Color : Background_window,
  //                c2 = sel ? Background_window : Select_Color;
  // DWIN_Draw_Rectangle(0, c1, 25, 279, 126, 318);
  // DWIN_Draw_Rectangle(0, c1, 24, 278, 127, 319);
  // DWIN_Draw_Rectangle(0, c2, 145, 279, 246, 318);
  // DWIN_Draw_Rectangle(0, c2, 144, 278, 247, 319);
}

void Popup_window_PauseOrStop(void) {
      Clear_Main_Window();

  if (HMI_flag.language_flag) {
   
    if (select_print.now == 1) 
    { 
      DWIN_ICON_Show(ICON,61,0,0);
      DWIN_ICON_Show(ICON,44,170,142);
      DWIN_ICON_Show(ICON,42,250,142);
    }
    else if (select_print.now == 2)
    { 
        DWIN_ICON_Show(ICON,57,0,0);
        DWIN_ICON_Show(ICON,44,170,142);
        DWIN_ICON_Show(ICON,42,250,142);
    }
  }
  else {
     DWIN_ICON_Show(ICON,37,0,0);
    if (select_print.now == 1) 
    {
        DWIN_ICON_Show(ICON,53,0,0);
        DWIN_ICON_Show(ICON,49,170,142);
        DWIN_ICON_Show(ICON,46,250,142);
    }
    else if (select_print.now == 2) 
    {
       DWIN_ICON_Show(ICON,50,0,0);
      DWIN_ICON_Show(ICON,49,170,142);
      DWIN_ICON_Show(ICON,46,250,142);
    }
    
  }
  Draw_Select_Highlight(true);
}

void Draw_Printing_Screen(void) {
    DWIN_ICON_Show(ICON,9,10,190);
    DWIN_Draw_IntValue(true, true, 0, MENU_FONT, Black, Background_black, 3, 40, 190, thermalManager.temp_hotend[0].celsius);
    DWIN_Draw_IntValue(true, true, 0, MENU_FONT, Black, Background_black, 3, 40 + 4 * STAT_CHR_W + 6, 190, thermalManager.temp_hotend[0].target);
    DWIN_Draw_String(false, false, MENU_FONT, Black, Background_black, 40 + 3 * STAT_CHR_W + 5, 190, (char*)"/");
    DWIN_ICON_Show(ICON,11,130,190);
    DWIN_Draw_IntValue(true, true, 0, MENU_FONT, Black, Background_black, 3, 140 + 2 * STAT_CHR_W, 190, feedrate_percentage);
    DWIN_Draw_String(false, false, MENU_FONT, Black, Background_black, 140 + 2 * STAT_CHR_W+25, 190, (char*)"%");
    DWIN_ICON_Show(ICON,10,10,220);
    DWIN_Draw_IntValue(true, true, 0, MENU_FONT, Black, Background_black, 3, 40, 220, thermalManager.temp_bed.celsius);
    DWIN_Draw_IntValue(true, true, 0, MENU_FONT, Black, Background_black, 3, 40 + 4 * STAT_CHR_W + 6, 220, thermalManager.temp_bed.target);
    DWIN_Draw_String(false, false, MENU_FONT, Black, Background_black, 40 + 3 * STAT_CHR_W + 5, 220, (char*)"/");
    DWIN_ICON_Show(ICON,12,130,220);
    show_plus_or_minus(MENU_FONT, Black,Background_black, 2, 2, 160, 220, BABY_Z_VAR * 100);
   
}

void Draw_Print_ProgressBar() {
  DWIN_ICON_Show(ICON, 22, 140, 84);
  DWIN_Draw_Rectangle(1, BarFill_Color, 140 + Percentrecord * 324 / 100, 84, 460, 103);
}

void Draw_Print_ProgressElapsed() {
  duration_t elapsed = print_job_timer.duration(); // print timer
  DWIN_Draw_IntValue(true, true, 1, font8x16, Black, White, 2, 140, 118, elapsed.value / 3600);
  DWIN_Draw_String(false, false, font8x16, Black, White, 140 + 16, 118, (char*)":");
  DWIN_Draw_IntValue(true, true, 1, font8x16, Black, White, 2, 140 + 24, 118, (elapsed.value % 3600) / 60);
   if (HMI_flag.language_flag)
  DWIN_Frame_AreaCopy(1,0,50,60,66,140,135);
  else
  DWIN_Frame_AreaCopy(1,0,65,96,85,140,135);

}

void Draw_Print_ProgressRemain() {
  DWIN_Draw_IntValue(true, true, 1, font8x16, Black, White, 2, 400, 118, remain_time / 3600);
  DWIN_Draw_String(false, false, font8x16, Black, White, 400 + 16, 118, (char*)":");
  DWIN_Draw_IntValue(true, true, 1, font8x16, Black, White, 2, 400 + 24, 118, (remain_time % 3600) / 60);
 if (HMI_flag.language_flag)
 {
  DWIN_Frame_AreaCopy(1,62,50,90,66,400,135);
  DWIN_Frame_AreaCopy(1,30,50,60,66,432,135);
 }
  else
  {
    DWIN_Frame_AreaCopy(1,98,65,150,85,400,135);
  }
  
}
inline void make_name_without_ext(char *dst, char *src, size_t maxlen=MENU_CHAR_LIMIT) {
  char * const name = card.longest_filename();
  size_t pos        = strlen(name); // index of ending nul

  // For files, remove the extension
  // which may be .gcode, .gco, or .g
  if (!card.flag.filenameIsDir)
    while (pos && src[pos] != '.') pos--; // find last '.' (stop at 0)

  size_t len = pos;   // nul or '.'
  if (len > maxlen) { // Keep the name short
    pos        = len = maxlen; // move nul down
    dst[--pos] = '.'; // insert dots
    dst[--pos] = '.';
    dst[--pos] = '.';
  }

  dst[len] = '\0';    // end it

  // Copy down to 0
  while (pos--) dst[pos] = src[pos];
}

void Goto_PrintProcess(void) {
  checkkey = PrintProcess;
  
  Clear_Main_Window();
  DWIN_Draw_Line(0X0000,140,40,450,40);
  DWIN_ICON_Show(ICON, ICON_LOGO, 20, 56);
  if (HMI_flag.language_flag)
  {
      DWIN_Frame_AreaCopy(1,160,2,206,18,273,12);
  }
  else
  {
      DWIN_Frame_AreaCopy(1,0,65,58,85,260,10);
  }
  Draw_Printing_Screen();
  DWIN_UpdateLCD();
  ICON_Tune();
  if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
  ICON_Stop();

  // Copy into filebuf string before entry
  char * const name = card.longest_filename();
  
  if(strlen(name)>12)
  {
      char temp_str[strlen(name) + 1];
      make_name_without_ext(temp_str, name);
      DWIN_Draw_String(false, false, font8x16, Black, Background_black, (DWIN_WIDTH - strlen(temp_str) ) / 2, 60, temp_str);
  }
  else
  {
     DWIN_Draw_String(false, false, font8x16, Black, Background_black, (DWIN_WIDTH - strlen(name)*4 ) / 2, 60, name);
  }
  
  SERIAL_ECHOLNPAIR("card.longest_filename()",card.longest_filename()); 
  Draw_Print_ProgressBar();
  Draw_Print_ProgressElapsed();
  Draw_Print_ProgressRemain();
}

void Goto_MainMenu(void) {
  checkkey = MainMenu;

  Clear_Main_Window();
  Draw_Steady_State();

  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 130, 2,160 ,16 , 285, 12); // "Home"
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Home"); // TODO: GET_TEXT
    #else
      DWIN_Frame_AreaCopy(1, 168, 2,212, 18, 275, 12);
    #endif
  }
  ICON_Print();
  ICON_Prepare();
  ICON_Control();
  ICON_StartInfo(select_page.now == 3);
  DWIN_UpdateLCD();
}

inline ENCODER_DiffState get_encoder_state() {
  const millis_t ms = millis();
  if (PENDING(ms, Encoder_ms)) return ENCODER_DIFF_NO;
  const ENCODER_DiffState state = Encoder_ReceiveAnalyze();
  if (state != ENCODER_DIFF_NO) Encoder_ms = ms + ENCODER_WAIT;
  return state;
}

void HMI_Move_X(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Move_X_scale += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Move_X_scale -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      checkkey = AxisMove;
      EncoderRate.encoderRateEnabled = 0;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(1), HMI_ValueStruct.Move_X_scale);
      if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_X_scale, (X_MIN_POS) * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_X_scale, (X_MAX_POS) * MINUNITMULT);
    current_position[X_AXIS] = HMI_ValueStruct.Move_X_scale / 10;
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(1), HMI_ValueStruct.Move_X_scale);
    DWIN_UpdateLCD();
  }
}

void HMI_Move_Y(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Move_Y_scale += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Move_Y_scale -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      checkkey = AxisMove;
      EncoderRate.encoderRateEnabled = 0;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(2), HMI_ValueStruct.Move_Y_scale);
      if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_Y_scale, (Y_MIN_POS) * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_Y_scale, (Y_MAX_POS) * MINUNITMULT);
    current_position[Y_AXIS] = HMI_ValueStruct.Move_Y_scale / 10;
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(2), HMI_ValueStruct.Move_Y_scale);
    DWIN_UpdateLCD();
  }
}

void HMI_Move_Z(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Move_Z_scale += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Move_Z_scale -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      checkkey = AxisMove;
      EncoderRate.encoderRateEnabled = 0;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(3), HMI_ValueStruct.Move_Z_scale);
      if (!planner.is_full()) {
        // Wait for planner moves to finish!
        planner.synchronize();
        planner.buffer_line(current_position, MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.Move_Z_scale, Z_MIN_POS * MINUNITMULT);
    NOMORE(HMI_ValueStruct.Move_Z_scale, Z_MAX_POS * MINUNITMULT);
    current_position[Z_AXIS] = HMI_ValueStruct.Move_Z_scale / 10;
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(3), HMI_ValueStruct.Move_Z_scale);
    DWIN_UpdateLCD();
  }
}

void HMI_Move_E(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
   char cmd[MAX_CMD_SIZE+16];
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Move_E_scale += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Move_E_scale -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      checkkey = AxisMove;
      EncoderRate.encoderRateEnabled = 0;
      // last_E_scale = HMI_ValueStruct.Move_E_scale;
      show_plus_or_minus(font8x16, Black,Background_black, 3, 1, 400, MBASE(4), HMI_ValueStruct.Move_E_scale);
      // if (!planner.is_full()) {
        // planner.synchronize(); // Wait for planner moves to finish!
        //planner.buffer_line(current_position, MMM_TO_MMS(FEEDRATE_E), active_extruder);
        //sprintf_P(cmd, PSTR("G0 E%s"), dtostrf(zraise, 1, 3, str_1));
        sprintf_P(cmd, PSTR("G1 E%s F50"),  HMI_ValueStruct.Move_E_scale/10+last_E_scale);
         gcode.process_subcommands_now(cmd);
        //    sprintf_P(cmd, PSTR("M109 S%i"), et);
        // gcode.process_subcommands_now(cmd);
      // }
      DWIN_UpdateLCD();
      return;
    }
    // if ((HMI_ValueStruct.Move_E_scale - last_E_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
    //   HMI_ValueStruct.Move_E_scale = last_E_scale + (EXTRUDE_MAXLENGTH) * MINUNITMULT;
    // else if ((last_E_scale - HMI_ValueStruct.Move_E_scale) > (EXTRUDE_MAXLENGTH) * MINUNITMULT)
    //   HMI_ValueStruct.Move_E_scale = last_E_scale - (EXTRUDE_MAXLENGTH) * MINUNITMULT;
    current_position.e = HMI_ValueStruct.Move_E_scale / 10+last_E_scale;
    show_plus_or_minus(font8x16, White,Select_Color, 3, 1, 400, MBASE(4), HMI_ValueStruct.Move_E_scale);
    DWIN_UpdateLCD();
  }
}

void HMI_Zoffset(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    last_zoffset = zprobe_zoffset;
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.offset_value += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.offset_value -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      EncoderRate.encoderRateEnabled = 0;
      zprobe_zoffset                 = HMI_ValueStruct.offset_value / 100;
      #if HAS_BED_PROBE
        if (WITHIN(zprobe_zoffset - last_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
          probe.offset.z = zprobe_zoffset;
        settings.save();
      #elif ENABLED(BABYSTEPPING)
        babystep.add_mm(Z_AXIS, (zprobe_zoffset - last_zoffset));
      #else
        UNUSED(zprobe_zoffset - last_zoffset);
      #endif

      if (HMI_ValueStruct.show_mode == -4) {
        checkkey = Prepare;
        show_plus_or_minus(font8x16,Black, Background_black, 2, 2, 400, MBASE(4 + MROWS - index_prepare), TERN(HAS_LEVELING, probe.offset.z * 100, HMI_ValueStruct.offset_value));
      }
      else {
        checkkey = Tune;
        show_plus_or_minus(font8x16,Black, Background_black, 2, 2, 400, MBASE(5 + MROWS - index_tune), TERN(HAS_LEVELING, probe.offset.z * 100, HMI_ValueStruct.offset_value));
      }
      DWIN_UpdateLCD();
      return;
    }
    NOLESS(HMI_ValueStruct.offset_value, (Z_PROBE_OFFSET_RANGE_MIN) * 100);
    NOMORE(HMI_ValueStruct.offset_value, (Z_PROBE_OFFSET_RANGE_MAX) * 100);
    if (HMI_ValueStruct.show_mode == -4)
      show_plus_or_minus(font8x16, White,Select_Color, 2, 2, 400, MBASE(4 + MROWS - index_prepare), HMI_ValueStruct.offset_value);
    else
      show_plus_or_minus(font8x16, White,Select_Color, 2, 2, 400, MBASE(5 + MROWS - index_tune), HMI_ValueStruct.offset_value);
    DWIN_UpdateLCD();
  }
}

#if HAS_HOTEND

  void HMI_ETemp(void) {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (encoder_diffState == ENCODER_DIFF_CW) {
        HMI_ValueStruct.E_Temp += EncoderRate.encoderMoveValue;
      }
      else if (encoder_diffState == ENCODER_DIFF_CCW) {
        HMI_ValueStruct.E_Temp -= EncoderRate.encoderMoveValue;
      }
      else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
        EncoderRate.encoderRateEnabled = 0;
        if (HMI_ValueStruct.show_mode == -1) { // temperature
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(1+MROWS - index_temperature), HMI_ValueStruct.E_Temp);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].hotend_temp = HMI_ValueStruct.E_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(1), ui.material_preset[0].hotend_temp);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].hotend_temp = HMI_ValueStruct.E_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(1), ui.material_preset[1].hotend_temp);
          return;
        }
        else { // tune
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(2 + MROWS - index_tune), HMI_ValueStruct.E_Temp);
        }
        thermalManager.setTargetHotend(HMI_ValueStruct.E_Temp, 0);
        return;
      }
      // E_Temp limit
      NOMORE(HMI_ValueStruct.E_Temp, MAX_E_TEMP);
      NOLESS(HMI_ValueStruct.E_Temp, MIN_E_TEMP);
      // E_Temp value
      if (HMI_ValueStruct.show_mode >= 0) // tune
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2 + MROWS - index_tune), HMI_ValueStruct.E_Temp);
      else if (HMI_ValueStruct.show_mode==-1) // other page
      {
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(1+MROWS - index_temperature), HMI_ValueStruct.E_Temp);
      }        
      else if(HMI_ValueStruct.show_mode==-2||HMI_ValueStruct.show_mode==-3)
      {
             DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(1), HMI_ValueStruct.E_Temp);
      }
    }
  }

#endif // if HAS_HOTEND

#if HAS_HEATED_BED

  void HMI_BedTemp(void) {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (encoder_diffState == ENCODER_DIFF_CW) {
        HMI_ValueStruct.Bed_Temp += EncoderRate.encoderMoveValue;
      }
      else if (encoder_diffState == ENCODER_DIFF_CCW) {
        HMI_ValueStruct.Bed_Temp -= EncoderRate.encoderMoveValue;
      }
      else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
        EncoderRate.encoderRateEnabled = 0;
        if (HMI_ValueStruct.show_mode == -1) {
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2+MROWS - index_temperature), HMI_ValueStruct.Bed_Temp);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].bed_temp = HMI_ValueStruct.Bed_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2), ui.material_preset[0].bed_temp);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].bed_temp = HMI_ValueStruct.Bed_Temp;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2), ui.material_preset[1].bed_temp);
          return;
        }
        else {
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3 + MROWS - index_tune), HMI_ValueStruct.Bed_Temp);
        }
        thermalManager.setTargetBed(HMI_ValueStruct.Bed_Temp);
        return;
      } 
      // Bed_Temp limit
      NOMORE(HMI_ValueStruct.Bed_Temp, BED_MAX_TARGET);
      NOLESS(HMI_ValueStruct.Bed_Temp, MIN_BED_TEMP);
      // Bed_Temp value
      if (HMI_ValueStruct.show_mode >= 0) // tune page
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3 + MROWS - index_tune), HMI_ValueStruct.Bed_Temp);
      else if(HMI_ValueStruct.show_mode==-1)// other page
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2+MROWS - index_temperature), HMI_ValueStruct.Bed_Temp);
      else if(HMI_ValueStruct.show_mode==-2||HMI_ValueStruct.show_mode==-3)
         DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2), HMI_ValueStruct.Bed_Temp);
    }
  }

#endif // if HAS_HEATED_BED

#if HAS_FAN

  void HMI_FanSpeed(void) {
    ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
    if (encoder_diffState != ENCODER_DIFF_NO) {
      if (encoder_diffState == ENCODER_DIFF_CW) {
        HMI_ValueStruct.Fan_speed += EncoderRate.encoderMoveValue;
      }
      else if (encoder_diffState == ENCODER_DIFF_CCW) {
        HMI_ValueStruct.Fan_speed -= EncoderRate.encoderMoveValue;
      }
      else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
        EncoderRate.encoderRateEnabled = 0;
        if (HMI_ValueStruct.show_mode == -1) {
          checkkey = TemperatureID;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3+MROWS-index_temperature), HMI_ValueStruct.Fan_speed);
        }
        else if (HMI_ValueStruct.show_mode == -2) {
          checkkey = PLAPreheat;
          ui.material_preset[0].fan_speed = HMI_ValueStruct.Fan_speed;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3), ui.material_preset[0].fan_speed);
          return;
        }
        else if (HMI_ValueStruct.show_mode == -3) {
          checkkey = ABSPreheat;
          ui.material_preset[1].fan_speed = HMI_ValueStruct.Fan_speed;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3), ui.material_preset[1].fan_speed);
          return;
        }
        else {
          checkkey = Tune;
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(4 + MROWS - index_tune), HMI_ValueStruct.Fan_speed);
        }
        thermalManager.set_fan_speed(0, HMI_ValueStruct.Fan_speed);
        return;
      }
      // Fan_speed limit
      NOMORE(HMI_ValueStruct.Fan_speed, FANON);
      NOLESS(HMI_ValueStruct.Fan_speed, FANOFF);
      // Fan_speed value
      if (HMI_ValueStruct.show_mode >= 0) // tune page
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(4 + MROWS - index_tune), HMI_ValueStruct.Fan_speed);
      else if (HMI_ValueStruct.show_mode==-1) // other page                            
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3+MROWS - index_temperature), HMI_ValueStruct.Fan_speed);
      else if(HMI_ValueStruct.show_mode == -2||HMI_ValueStruct.show_mode==-3)
         DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3), HMI_ValueStruct.Fan_speed);
    }
  }

#endif // if HAS_FAN

void HMI_PrintSpeed(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.print_speed += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.print_speed -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
      checkkey = Tune;
      EncoderRate.encoderRateEnabled = 0;
      feedrate_percentage            = HMI_ValueStruct.print_speed;
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(1 + MROWS - index_tune), HMI_ValueStruct.print_speed);
      return;
    }
    // print_speed limit
    NOMORE(HMI_ValueStruct.print_speed, MAX_PRINT_SPEED);
    NOLESS(HMI_ValueStruct.print_speed, MIN_PRINT_SPEED);
    // print_speed value
    DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(1 + MROWS - index_tune), HMI_ValueStruct.print_speed);
  }
}

void HMI_MaxFeedspeedXYZE(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Max_Feedspeed += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Max_Feedspeed -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
      checkkey = MaxSpeed;
      EncoderRate.encoderRateEnabled = 0;
      if (HMI_flag.feedspeed_flag == X_AXIS) 
      {
        planner.set_max_feedrate(X_AXIS, HMI_ValueStruct.Max_Feedspeed);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(1 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
      }
      else if (HMI_flag.feedspeed_flag == Y_AXIS) 
       {
        planner.set_max_feedrate(Y_AXIS, HMI_ValueStruct.Max_Feedspeed);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(2 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
      }
      else if (HMI_flag.feedspeed_flag == Z_AXIS) 
       {
        planner.set_max_feedrate(Z_AXIS, HMI_ValueStruct.Max_Feedspeed);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(3 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
      }
      #if HAS_HOTEND
        else if (HMI_flag.feedspeed_flag == E_AXIS) 
         {
             planner.set_max_feedrate(E_AXIS, HMI_ValueStruct.Max_Feedspeed);
              DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(4 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
        }
      #endif
      return;                                                                            
    }
    // MaxFeedspeed limit
    if (HMI_flag.feedspeed_flag == X_AXIS)
     {if (HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[X_AXIS] * 2)
        HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[X_AXIS] * 2; 
         if (HMI_ValueStruct.Max_Feedspeed < MIN_MAXFEEDSPEED) HMI_ValueStruct.Max_Feedspeed = MIN_MAXFEEDSPEED;
        DWIN_Draw_IntValue(true, true, 0, font8x16,White, Select_Color, 4, 400, MBASE(1 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
      }
    else if (HMI_flag.feedspeed_flag == Y_AXIS) 
    {
      if (HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[Y_AXIS] * 2)
       HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[Y_AXIS] * 2; 
      if (HMI_ValueStruct.Max_Feedspeed < MIN_MAXFEEDSPEED) HMI_ValueStruct.Max_Feedspeed = MIN_MAXFEEDSPEED;
      DWIN_Draw_IntValue(true, true, 0, font8x16,White, Select_Color, 4, 400, MBASE(2 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
     }
    else if (HMI_flag.feedspeed_flag == Z_AXIS) 
    {
      if (HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[Z_AXIS] * 2) 
         HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[Z_AXIS] * 2; 
        if (HMI_ValueStruct.Max_Feedspeed < MIN_MAXFEEDSPEED) HMI_ValueStruct.Max_Feedspeed = MIN_MAXFEEDSPEED;
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(3 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
    }
    #if HAS_HOTEND
      else if (HMI_flag.feedspeed_flag == E_AXIS) 
      {
        if (HMI_ValueStruct.Max_Feedspeed > default_max_feedrate[E_AXIS] * 2)
         HMI_ValueStruct.Max_Feedspeed = default_max_feedrate[E_AXIS] * 2; 
         DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(4 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
         }
    #endif
  }
}

void HMI_MaxAccelerationXYZE(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) { HMI_ValueStruct.Max_Acceleration += EncoderRate.encoderMoveValue;}
    else if (encoder_diffState == ENCODER_DIFF_CCW) { HMI_ValueStruct.Max_Acceleration -= EncoderRate.encoderMoveValue;}
    else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
      checkkey = MaxAcceleration;
      EncoderRate.encoderRateEnabled = 0;
      if (HMI_flag.acc_flag == X_AXIS) 
      {
         planner.set_max_acceleration(X_AXIS, HMI_ValueStruct.Max_Acceleration);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(1 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
      }
      else if (HMI_flag.acc_flag == Y_AXIS)
      {
         planner.set_max_acceleration(Y_AXIS, HMI_ValueStruct.Max_Acceleration);
         DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(2 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
      }
      else if (HMI_flag.acc_flag == Z_AXIS) 
      {
        planner.set_max_acceleration(Z_AXIS, HMI_ValueStruct.Max_Acceleration);
        DWIN_Draw_IntValue(true, true, 0, font8x16,Black, Background_black, 4, 400, MBASE(3 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
      }
      #if HAS_HOTEND
        else if (HMI_flag.acc_flag == E_AXIS) 
        {
          planner.set_max_acceleration(E_AXIS, HMI_ValueStruct.Max_Acceleration);
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(4 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
        }
      #endif
      return;
    }
    // MaxAcceleration limit
    if (HMI_flag.acc_flag == X_AXIS) 
    {
      if (HMI_ValueStruct.Max_Acceleration > default_max_acceleration[X_AXIS] * 2) 
      HMI_ValueStruct.Max_Acceleration = default_max_acceleration[X_AXIS] * 2;
      if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
    // MaxAcceleration value
      DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(1 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
    }
    else if (HMI_flag.acc_flag == Y_AXIS)
    {
      if (HMI_ValueStruct.Max_Acceleration > default_max_acceleration[Y_AXIS] * 2) 
      HMI_ValueStruct.Max_Acceleration = default_max_acceleration[Y_AXIS] * 2; 
      if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
    // MaxAcceleration value
      DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(2 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
    }
    else if (HMI_flag.acc_flag == Z_AXIS)
     {
        if (HMI_ValueStruct.Max_Acceleration > default_max_acceleration[Z_AXIS] * 2) 
        HMI_ValueStruct.Max_Acceleration = default_max_acceleration[Z_AXIS] * 2;
        if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
    // MaxAcceleration value
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(3 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
      }
    #if HAS_HOTEND
      else if (HMI_flag.acc_flag == E_AXIS) 
      {
        if (HMI_ValueStruct.Max_Acceleration > default_max_acceleration[E_AXIS] * 2) 
        HMI_ValueStruct.Max_Acceleration = default_max_acceleration[E_AXIS] * 2; 
        if (HMI_ValueStruct.Max_Acceleration < MIN_MAXACCELERATION) HMI_ValueStruct.Max_Acceleration = MIN_MAXACCELERATION;
    // MaxAcceleration value
      DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(4 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
      }
    #endif
  }
}

void HMI_MaxCornerXYZE(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Max_Corner += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Max_Corner -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
      checkkey = MaxCorner;
      EncoderRate.encoderRateEnabled = 0;
      if (HMI_flag.corner_flag == X_AXIS)
      {
        planner.set_max_jerk(X_AXIS, HMI_ValueStruct.Max_Corner / 10);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(1 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
      }
      else if (HMI_flag.corner_flag == Y_AXIS) 
      {
        planner.set_max_jerk(Y_AXIS, HMI_ValueStruct.Max_Corner / 10);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(2 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
      }
      else if (HMI_flag.corner_flag == Z_AXIS) 
      {
        planner.set_max_jerk(Z_AXIS, HMI_ValueStruct.Max_Corner / 10);
       DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(3 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
      }
      else if (HMI_flag.corner_flag == E_AXIS) 
      {
        planner.set_max_jerk(E_AXIS, HMI_ValueStruct.Max_Corner / 10);
             DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(4 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
      }
      return;
    }
    // MaxCorner limit
    if (HMI_flag.corner_flag == X_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Corner, default_max_jerk[X_AXIS] * 2 * MINUNITMULT);
       NOLESS(HMI_ValueStruct.Max_Corner, (MIN_MAXCORNER) * MINUNITMULT);
    // MaxCorner value
      DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(1 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
    }
    else if (HMI_flag.corner_flag == Y_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Corner, default_max_jerk[Y_AXIS] * 2 * MINUNITMULT);
       NOLESS(HMI_ValueStruct.Max_Corner, (MIN_MAXCORNER) * MINUNITMULT);
    // MaxCorner value
      DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(2 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
    }
    else if (HMI_flag.corner_flag == Z_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Corner, default_max_jerk[Z_AXIS] * 2 * MINUNITMULT);
      NOLESS(HMI_ValueStruct.Max_Corner, (MIN_MAXCORNER) * MINUNITMULT);
    // MaxCorner value
      DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(3 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
    }
    else if (HMI_flag.corner_flag == E_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Corner, default_max_jerk[E_AXIS] * 2 * MINUNITMULT);
      NOLESS(HMI_ValueStruct.Max_Corner, (MIN_MAXCORNER) * MINUNITMULT);
    // MaxCorner value
      DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(4 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
    }
  }
}

void HMI_StepXYZE(void) {
  ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
  if (encoder_diffState != ENCODER_DIFF_NO) {
    if (encoder_diffState == ENCODER_DIFF_CW) {
      HMI_ValueStruct.Max_Step += EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      HMI_ValueStruct.Max_Step -= EncoderRate.encoderMoveValue;
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) { // return
      checkkey = Step;
      EncoderRate.encoderRateEnabled = 0;

      if (HMI_flag.step_flag == X_AXIS) 
      { 
        planner.settings.axis_steps_per_mm[X_AXIS] = HMI_ValueStruct.Max_Step / 10;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(1 + MROWS -index_step), HMI_ValueStruct.Max_Step);
      }
      else if (HMI_flag.step_flag == Y_AXIS) 
      {
        planner.settings.axis_steps_per_mm[Y_AXIS] = HMI_ValueStruct.Max_Step / 10;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(2 + MROWS -index_step), HMI_ValueStruct.Max_Step);
      }
      else if (HMI_flag.step_flag == Z_AXIS) 
      {
      planner.settings.axis_steps_per_mm[Z_AXIS] = HMI_ValueStruct.Max_Step / 10;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(3 + MROWS -index_step), HMI_ValueStruct.Max_Step);
      }
      else if (HMI_flag.step_flag == E_AXIS) 
     {
      planner.settings.axis_steps_per_mm[E_AXIS] = HMI_ValueStruct.Max_Step / 10;
      DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(4 + MROWS -index_step), HMI_ValueStruct.Max_Step);
      }
      return;
    }
    // Step limit
    if (HMI_flag.step_flag == X_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[X_AXIS] * 2 * MINUNITMULT);
      NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP);
    // // Step value
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(1 + MROWS -index_step), HMI_ValueStruct.Max_Step);
    }
    else if (HMI_flag.step_flag == Y_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[Y_AXIS] * 2 * MINUNITMULT);
     NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP);
    // Step value
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(2 + MROWS -index_step), HMI_ValueStruct.Max_Step);
    
    }
    else if (HMI_flag.step_flag == Z_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[Z_AXIS] * 2 * MINUNITMULT);
     NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP);
    // Step value
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(3 + MROWS -index_step), HMI_ValueStruct.Max_Step);
    }
    else if (HMI_flag.step_flag == E_AXIS)
    {
      NOMORE(HMI_ValueStruct.Max_Step, default_axis_steps_per_unit[E_AXIS] * 2 * MINUNITMULT);
      NOLESS(HMI_ValueStruct.Max_Step, MIN_STEP);
    // Step value
    DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(4 + MROWS -index_step), HMI_ValueStruct.Max_Step);
    }
  }
}                                                                                 

void update_variable(void) {
  /* Tune page temperature update */
  if (checkkey == Tune) {
    if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2 + MROWS - index_tune), thermalManager.temp_hotend[0].target);
    if (last_temp_bed_target != thermalManager.temp_bed.target)
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3 + MROWS - index_tune), thermalManager.temp_bed.target);
    if (last_fan_speed != thermalManager.fan_speed[0]) {
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(4 + MROWS - index_tune), thermalManager.fan_speed[0]);
      last_fan_speed = thermalManager.fan_speed[0];
    }
  }

  /* Temperature page temperature update */
  if (checkkey == TemperatureID) {
    if (last_temp_hotend_target != thermalManager.temp_hotend[0].target)
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(1+MROWS-index_temperature), thermalManager.temp_hotend[0].target);
    if (last_temp_bed_target != thermalManager.temp_bed.target)
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2+MROWS-index_temperature), thermalManager.temp_bed.target);
    if (last_fan_speed != thermalManager.fan_speed[0]) {
      DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3+MROWS-index_temperature), thermalManager.fan_speed[0]);
      last_fan_speed = thermalManager.fan_speed[0];
    }
  }
    if(checkkey== PrintProcess)
    {
        /* Bottom temperature update */
      if (last_temp_hotend_current != thermalManager.temp_hotend[0].celsius) {
        //DWIN_Draw_IntValue(true, true, 0, STAT_FONT, Black, Background_black, 3, 33, 382, thermalManager.temp_hotend[0].celsius);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 40, 190, thermalManager.temp_hotend[0].celsius);
        last_temp_hotend_current = thermalManager.temp_hotend[0].celsius;
      }
      if (last_temp_hotend_target != thermalManager.temp_hotend[0].target) {
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 40 + 4 * STAT_CHR_W + 6, 190, thermalManager.temp_hotend[0].target);
        last_temp_hotend_target = thermalManager.temp_hotend[0].target;
      }
      if (last_temp_bed_current != thermalManager.temp_bed.celsius) {
        // DWIN_Draw_IntValue(true, true, 0, STAT_FONT, Black, Background_black, 3, 178, 382, thermalManager.temp_bed.celsius);
        DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 40, 220, thermalManager.temp_bed.celsius);
        last_temp_bed_current = thermalManager.temp_bed.celsius;
      }
      if (last_temp_bed_target != thermalManager.temp_bed.target) {
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 40 + 4 * STAT_CHR_W + 6, 220, thermalManager.temp_bed.target);
        last_temp_bed_target = thermalManager.temp_bed.target;
      }
      if (last_speed != feedrate_percentage) {
          DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 140 + 2 * STAT_CHR_W, 190, feedrate_percentage);
        last_speed = feedrate_percentage;
      }
  #if HAS_LEVELING
    if (last_probe_zoffset != probe.offset.z) {
       show_plus_or_minus(MENU_FONT,Black, Background_black, 2, 2, 160, 220, probe.offset.z * 100);
      last_probe_zoffset = probe.offset.z;
    }
  #else
    if (last_zoffset != zprobe_zoffset) {
      show_plus_or_minus(MENU_FONT,Black, Background_black, 2, 2, 160, 220, BABY_Z_VAR * 100);
      last_zoffset = zprobe_zoffset;
    }
  #endif
  }
   else
   {
          if(checkkey!=Last_Control&&checkkey!=Back_Main&&HMI_flag.ETempTooLow_flag!=1 && checkkey != Print_window && checkkey !=filament && checkkey!=filament_exist)//
          {
            /* Bottom temperature update */
              if (last_temp_hotend_current != thermalManager.temp_hotend[0].celsius) {
                DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 50, 200, thermalManager.temp_hotend[0].celsius);
                last_temp_hotend_current = thermalManager.temp_hotend[0].celsius;
              }
               if (last_temp_hotend_target != thermalManager.temp_hotend[0].target) {
                  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 50 + 4 * STAT_CHR_W + 6, 200, thermalManager.temp_hotend[0].target);
                  last_temp_hotend_target = thermalManager.temp_hotend[0].target;
                }
              if (last_temp_bed_current != thermalManager.temp_bed.celsius) {
                DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 50, 236, thermalManager.temp_bed.celsius);
                last_temp_bed_current = thermalManager.temp_bed.celsius;
              }
              if (last_temp_bed_target != thermalManager.temp_bed.target) {
                 DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 50 + 4 * STAT_CHR_W + 6, 236, thermalManager.temp_bed.target);
                 last_temp_bed_target = thermalManager.temp_bed.target;
              }
          }
   }
}

/**
* Read and cache the working directory.
*
* TODO: New code can follow the pattern of menu_media.cpp
* and rely on Marlin caching for performance. No need to
* cache files here.
*
*/

#ifndef strcasecmp_P
  #define strcasecmp_P(a, b) strcasecmp((a), (b))
#endif

// inline void make_name_without_ext(char *dst, char *src, int maxlen=MENU_CHAR_LIMIT) {
//   char * const name = card.longest_filename();
//   size_t pos        = strlen(name); // index of ending nul

//   // For files, remove the extension
//   // which may be .gcode, .gco, or .g
//   if (!card.flag.filenameIsDir)
//     while (pos && src[pos] != '.') pos--; // find last '.' (stop at 0)

//   int len = pos;      // nul or '.'
//   if (len > maxlen) { // Keep the name short
//     pos        = len = maxlen; // move nul down
//     dst[--pos] = '.'; // insert dots
//     dst[--pos] = '.';
//     dst[--pos] = '.';
//   }

//   dst[len] = '\0';    // end it

//   // Copy down to 0
//   while (pos--) dst[pos] = src[pos];
// }

inline void HMI_SDCardInit(void) { card.cdroot(); }

void MarlinUI::refresh() {
  // The card was mounted or unmounted
  // or some other status change occurred
  // DWIN_lcd_sd_status = false; // On next DWIN_Update
  // HMI_SDCardUpdate();
}

#define ICON_Folder ICON_More

char shift_name[LONG_FILENAME_LENGTH + 1];
int8_t shift_amt; // = 0
millis_t shift_ms; // = 0

// Init the shift name based on the highlighted item
inline void Init_Shift_Name() {
  const bool is_subdir = !card.flag.workDirIsRoot;
  const int8_t filenum = select_file.now - 1 - is_subdir; // Skip "Back" and ".."
  const uint16_t fileCnt = card.get_num_Files();
  if (WITHIN(filenum, 0, fileCnt - 1)) {
    card.getfilename_sorted(SD_ORDER(filenum, fileCnt));
    char * const name = card.longest_filename();
    make_name_without_ext(shift_name, name, 100);
  }
}

inline void Init_SDItem_Shift() {
  shift_amt = 0;
  shift_ms  = select_file.now > 0 && strlen(shift_name) > MENU_CHAR_LIMIT
         ? millis() + 750UL : 0;
}

/**
 * Display an SD item, adding a CDUP for subfolders.
 */
inline void Draw_SDItem(const uint16_t item, int16_t row=-1) {
  if (row < 0) row = item + 1 + MROWS - index_file;
  const bool is_subdir = !card.flag.workDirIsRoot;
  if (is_subdir && item == 0) {
    Draw_Menu_Line(row, ICON_Folder, (char*)"..");
    return;
  }

  card.getfilename_sorted(item - is_subdir);
  char * const name = card.longest_filename();

  // Init the current selected name
  // This is used during scroll drawing
    #if ENABLED(SCROLL_LONG_FILENAMES)
    // Init the current selected name
    // This is used during scroll drawing
    if (item == select_file.now - 1) {
      make_name_without_ext(shift_name, name, 100);
      Init_SDItem_Shift();
    }
  #endif

  char str[strlen(name) + 1];

  make_name_without_ext(str, name,24);
  if(card.flag.filenameIsDir)
  {
    return;
  }
  else
  {
    /* code */
    Draw_Menu_Line(row, card.flag.filenameIsDir ? ICON_Folder : ICON_File, str);
     //DWIN_Draw_String(false, false, font8x16, Black, Background_black, LBLX, 48 + row * MLINE, (char*)str);
  }
}

inline void Draw_SDItem_Shifted(int8_t &shift) {
  // Limit to the number of chars past the cutoff
  const size_t len = strlen(shift_name);
  NOMORE(shift, _MAX((signed)len - MENU_CHAR_LIMIT, 0));

  // Shorten to the available space
  const size_t lastchar = _MIN((signed)len, shift + MENU_CHAR_LIMIT);

  const char c = shift_name[lastchar];
  shift_name[lastchar] = '\0';

  const uint8_t row = select_file.now + MROWS - index_file; // skip "Back" and scroll
  Erase_Menu_Text(row);
  Draw_Menu_Line(row, 0, &shift_name[shift]);

  shift_name[lastchar] = c;
}

// Redraw the first set of SD Files
inline void Redraw_SD_List() {
  select_file.reset();
  index_file = MROWS;
  Clear_Menu_Area(); // Leave title bar unchanged
  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 310, 20, 368, 36, 265, 12);
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Print file"); // TODO: GET_TEXT_F
    #else
      //DWIN_Frame_AreaCopy(1, 52, 31, 271 - 134, 479 - 438, 14, 8); // "Print file"
      DWIN_Frame_AreaCopy(1,0,250,32,270,265,12);
      DWIN_Frame_AreaCopy(1,110,50,136,68,265+32+8,12);
    #endif
  }
  Draw_Steady_State();
  Draw_Back_First();
  DWIN_UpdateLCD();
  if(card.isMounted())
  {
      TERN_(SDCARD_SORT_ALPHA,card.presort());
  }
  // As many files as will fit
  if (card.isMounted()) {
    // As many files as will fit
    LOOP_L_N(i, _MIN(nr_sd_menu_items(), MROWS))
      Draw_SDItem(i, i + 1);

    TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
  }
}

inline void SDCard_Up(void) {
  card.cdup();
  Redraw_SD_List();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

inline void SDCard_Folder(char * const dirname) {
  card.cd(dirname);
  Redraw_SD_List();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

//
// Watch for media mount / unmount
//
void HMI_SDCardUpdate() {
  if (HMI_flag.home_flag) return;
  if (DWIN_lcd_sd_status != card.isMounted()) {
    DWIN_lcd_sd_status = card.isMounted();
    if (DWIN_lcd_sd_status) {
      if (checkkey == SelectFile)
        Redraw_SD_List();
    }
    else {
      // clean file icon
      if (checkkey == SelectFile) {
        Redraw_SD_List();
      }
      else if (checkkey == PrintProcess || checkkey == Tune || printingIsActive()) {
        // TODO: Move card removed abort handling
        //       to CardReader::manage_media.
        card.flag.abort_sd_printing = true;
        wait_for_heatup = wait_for_user = false;
        abort_flag = true; // Reset feedrate, return to Home
      }
    }
    // DWIN_UpdateLCD();
  }
}

/* Start */
void HMI_StartFrame(const bool with_update) {
  Goto_MainMenu();
  if (with_update) {
    DWIN_UpdateLCD();
    delay(5);
  }
}

inline void Draw_Info_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
  DWIN_Draw_String(false, false, font8x16, Black, White, (DWIN_WIDTH - 36 * MENU_CHR_W) , 60, (char*)"Voxelab Aquila");
  DWIN_Draw_String(false, false, font8x16, Black, White, (DWIN_WIDTH - 36 * MENU_CHR_W) , 90, (char*)"220*220*250 mm");
  DWIN_Draw_String(false, false, font8x16, Black, White, (DWIN_WIDTH - 18 * MENU_CHR_W) , 120, (char*)SHORT_BUILD_VERSION);
  DWIN_Draw_String(false, false, font8x16, Black, White, (DWIN_WIDTH - 44 * MENU_CHR_W) , 150, (char*)"www.voxelab3dp.com");
  DWIN_Draw_String(false, false, font8x16, Black, White, (DWIN_WIDTH - 48 * MENU_CHR_W) , 180, (char*)"sales@voxelab3dp.com");

  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 338, 68,368 ,84, 285, 12); // "Info"
    DWIN_Frame_AreaCopy(1,340,122,400,138,150,60);
    DWIN_Frame_AreaCopy(1,246,104,305,120,150,90);
    DWIN_Frame_AreaCopy(1,308,104,365,120,150,120);
    DWIN_Frame_AreaCopy(1,400,122,430,138,150,150);
    DWIN_Frame_AreaCopy(1,432,122,470,138,150,180);
    DWIN_ICON_Show(ICON,25,160,225);                 //返回箭头
    DWIN_Frame_AreaCopy(1, 124, 50, 150,64, 170,225);  //返回
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Info"); // TODO: GET_TEXT_F
    #else
      DWIN_Frame_AreaCopy(1, 148, 100, 178, 120, 275, 12);
    #endif
     DWIN_Frame_AreaCopy(1, 340, 166, 400,180, LBLX, MBASE(4)-10);
     DWIN_Frame_AreaCopy(1,0,220,100,240,LBLX-20, 60);
     DWIN_Frame_AreaCopy(1,110,145,140,165,LBLX-20, 90);
     DWIN_Frame_AreaCopy(1,142,145,270,165,LBLX-20, 120);
     DWIN_Frame_AreaCopy(1,105,220,162,240,LBLX-20, 150);
     DWIN_Frame_AreaCopy(1,166,220,225,240,LBLX-20, 180);
     
    // DWIN_Frame_AreaCopy(1, 120, 150, 146, 479 - 318, 124, 102);
    // DWIN_Frame_AreaCopy(1, 146, 151, 271 - 17, 479 - 318, 82, 175);
    // DWIN_Frame_AreaCopy(1, 0, 165, 271 - 177, 479 - 304, 89, 248);
    // DWIN_Draw_String(false, false, font8x16, Black, White, (DWIN_WIDTH - strlen(CORP_WEBSITE_E) * MENU_CHR_W) / 2, 268, (char*)CORP_WEBSITE_E);
  }

  DWIN_Draw_Line(0X0000,140,210,450,210);
  DWIN_Draw_Rectangle(1, Red_Color, 140, 210,150,250);
  DWIN_Draw_Line(0X0000,140,250,450,250);
  

}

inline void Draw_Print_File_Menu() {
  // Clear_Title_Bar();
    Clear_Main_Window();
   
  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 310, 20, 368, 36, 265, 12);
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Print file"); // TODO: GET_TEXT_F
    #else
      DWIN_Frame_AreaCopy(1, 52, 31, 271 - 134, 479 - 438, 14, 8); // "Print file"
    #endif
  }
  Redraw_SD_List();
}

/* Main Process */
void HMI_MainMenu(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_page.inc(3)) {
      switch (select_page.now) {
        case 0:    ICON_Print();  break;
        case 1:    ICON_Print();ICON_Control(); break;
        case 2:    ICON_Control(); ICON_Prepare(); break;
        case 3:    ICON_Prepare(); ICON_StartInfo(1); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_page.dec()) {
      switch (select_page.now) {
          case 0: ICON_Print();ICON_Control(); break;
          case 1: ICON_Control(); ICON_Prepare(); break;
          case 2: ICON_Prepare(); ICON_StartInfo(0); break;
          case 3: ICON_StartInfo(1); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_page.now) {
      /* Print File */
      case 0:
        checkkey = SelectFile;
        Draw_Print_File_Menu();
        break;
      /* Prepare */
      case 1:
        checkkey = Control;
        index_control = MROWS;
        select_control.reset();
        Draw_Control_Menu();
        break;
      /* Control */
      case 2:
        checkkey = Prepare;
        select_prepare.reset();
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        //HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
        break;

      /* Leveling */
      case 3:
        #if 0
          checkkey = Leveling;
          HMI_Leveling();
        #else
          checkkey = Info;
          Draw_Info_Menu();
        #endif
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Select (and Print) File */
void HMI_SelectFile(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();

  const uint16_t hasUpDir = !card.flag.workDirIsRoot;

  if (encoder_diffState == ENCODER_DIFF_NO) {
    #if ENABLED(SCROLL_LONG_FILENAMES)
    if (shift_ms && select_file.now >= 1 + hasUpDir) {
      //Scroll selected filename every second
      const millis_t ms = millis();
      if (ELAPSED(ms, shift_ms)) {
        const bool was_reset = shift_amt < 0;
        shift_ms = ms + 375UL + was_reset * 250UL;  // ms per character
        int8_t shift_new = shift_amt + 1;           // Try to shift by...
        Draw_SDItem_Shifted(shift_new);             // Draw the item
        if (!was_reset && shift_new == 0)           // Was it limited to 0?
          shift_ms = 0;                             // No scrolling needed
        else if (shift_new == shift_amt)            // Scroll reached the end
          shift_new = -1;                           // Reset
        shift_amt = shift_new;                      // Set new scroll
      }
    }
    #endif
    return;
  }
  // First pause is long. Easy.
  // On reset, long pause must be after 0.

  uint16_t fullCnt = nr_sd_menu_items();
  //SERIAL_ECHOLNPAIR("fullCnt",fullCnt);
  // if(fullCnt>=SDSORT_LIMIT)
  // {
  //     fullCnt=SDSORT_LIMIT;
  // }
  if (encoder_diffState == ENCODER_DIFF_CW && fullCnt) {
    if (select_file.inc(fullCnt)) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        Erase_Menu_Text(itemnum + MROWS - index_file);          // Erase and
        Draw_SDItem(itemnum - 1);                                  // redraw
      }
      if (select_file.now > MROWS && select_file.now > index_file) { // Cursor past the bottom
        index_file = select_file.now;                           // New bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        Draw_SDItem(itemnum, MROWS);                            // Draw and init the shift name
      }
      else {
        Move_Highlight(1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());         // ...and init the shift name                                     // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());
    }
   
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && fullCnt) {
    if (select_file.dec()) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
     if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {                                         // If line was shifted
        Erase_Menu_Text(select_file.now + 1 + MROWS - index_file); // Erase and
        Draw_SDItem(itemnum + 1);                               // redraw
      }
      if (select_file.now < index_file - MROWS) {               // Cursor past the top
        index_file--;                                           // New bottom line
        Scroll_Menu(DWIN_SCROLL_DOWN);
        if (index_file == MROWS) {
          Draw_Back_First();
          TERN_(SCROLL_LONG_FILENAMES, shift_ms = 0);
        }
        else {
          Draw_SDItem(itemnum, 0);                              // Draw the item (and init shift name)
        }
      }
      else {
        Move_Highlight(-1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, Init_Shift_Name());                                   // ...and init the shift name
      }
     TERN_(SCROLL_LONG_FILENAMES, Init_SDItem_Shift());        // Reset left. Init timer.                        // Reset left. Init timer.
    }
  
  }

  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_file.now == 0) {
      /* back */
      select_page.set(0);
      Goto_MainMenu();
    }
    else if (hasUpDir && select_file.now == 1) {
      /* CDUP */
      SDCard_Up();
      goto HMI_SelectFileExit;
    }
    else {
      const uint16_t filenum = select_file.now - 1 - hasUpDir;
      card.getfilename_sorted(SD_ORDER(filenum, card.get_num_Files()));
      // SERIAL_ECHOLNPAIR("filenum",filenum);
      // SERIAL_ECHOLNPAIR("card.filename",card.filename);
      // SERIAL_ECHOLNPAIR("filenum123",SD_ORDER(filenum, card.get_num_Files()));
      // Enter that folder!
      // if (card.flag.filenameIsDir) {
      //   SDCard_Folder(card.filename);
      //   goto HMI_SelectFileExit;
      // }

      // Reset highlight for next entry
      select_print.reset();
      select_file.reset();
      //SERIAL_ECHOLNPAIR("START",card.filename);
      filament_state = false;
      filament_select = true;
      // Start choice and print SD file
      HMI_flag.heat_flag        = 1;
      HMI_flag.print_finish     = 0;
      HMI_ValueStruct.show_mode = 0;
      card.openAndPrintFile(card.filename);
     

      #if FAN_COUNT > 0
        // All fans on for Ender 3 v2 ?
        // The slicer should manage this for us.
        for (uint8_t i = 0; i < FAN_COUNT; i++)
         thermalManager.fan_speed[i] = FANON;
      #endif
      Percentrecord = 0;
      Draw_Print_ProgressBar();
      Goto_PrintProcess();
    }
  }
HMI_SelectFileExit:
  DWIN_UpdateLCD();
}

/* Printing */
void HMI_Printing(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // if (HMI_flag.confirm_flag) {
  //   if (encoder_diffState == ENCODER_DIFF_ENTER) {
  //     HMI_flag.confirm_flag = 0;
  //     abort_flag            = 1;
  //   }
  //   return;
  // }
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_print.inc(2)) {
      switch (select_print.now) {
        case 0: ICON_Tune(); break;
        case 1:
          ICON_Tune();
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          break;
        case 2:
           if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          ICON_Stop();
          break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_print.dec()) {
      switch (select_print.now) {
        case 0:
          ICON_Tune();
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          break;
        case 1:
          if (printingIsPaused()) ICON_Continue(); else ICON_Pause();
          ICON_Stop();
          break;
        case 2: ICON_Stop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_print.now) {
      case 0: // setting
        checkkey = Tune;
        HMI_ValueStruct.show_mode = 0;
        select_tune.reset();
        index_tune = MROWS;
        Draw_Tune_Menu();
        break;
      case 1: // pause
        /* pause */
        if(filament_select==true)
        {
          if (HMI_flag.pause_flag) {
            ICON_Pause();
            char cmd[40];
            cmd[0] = '\0';

            #if ENABLED(PAUSE_HEAT)
              if (tempbed) sprintf_P(cmd, PSTR("M190 S%i\n"), tempbed);
              if (temphot) sprintf_P(&cmd[strlen(cmd)], PSTR("M109 S%i\n"), temphot);
            #endif

            strcat_P(cmd, PSTR("M24"));
            queue.inject(cmd);
          }
          else {
            HMI_flag.select_flag = 1;
            checkkey = Print_window;
          Popup_window_PauseOrStop();
          }
        }
        break;

      case 2: // stop
          /* stop */
        HMI_flag.select_flag = 1;
        checkkey = Print_window;
        Popup_window_PauseOrStop();
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* pause and stop window */
void HMI_PauseOrStop(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
     Draw_Select_Highlight(false);
     if(HMI_flag.language_flag)
     {
          DWIN_ICON_Show(ICON,43,250,142);
          DWIN_ICON_Show(ICON,45,170,142);
     }
     else
     {
          DWIN_ICON_Show(ICON,48,170,142);
          DWIN_ICON_Show(ICON,47,250,142);
     }
     
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    Draw_Select_Highlight(true);
    if(HMI_flag.language_flag)
    {
        DWIN_ICON_Show(ICON,44,170,142);
        DWIN_ICON_Show(ICON,42,250,142);
    }
    else
    {
        DWIN_ICON_Show(ICON,49,170,142);
        DWIN_ICON_Show(ICON,46,250,142);  
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
     if (select_print.now == 1) { // pause window
      if (HMI_flag.select_flag) {
        pause_action_flag = 1;
        ICON_Continue();
        #if ENABLED(POWER_LOSS_RECOVERY)
          if (recovery.enabled) recovery.save(true);
        #endif
        queue.inject_P(PSTR("M25"));
        //
      }
      else {
        // cancel pause
      }
      Goto_PrintProcess();
    }
    else if (select_print.now == 2) { // stop window
      if (HMI_flag.select_flag) {
        wait_for_heatup = false; // Stop waiting for heater
        #if 1
          // TODO: In ExtUI or MarlinUI add a common stop event
           checkkey = Back_Main;
         
           card.endFilePrint();
           TERN_(POWER_LOSS_RECOVERY, recovery.cancel());
           card.flag.abort_sd_printing = true;
           Popup_Window_Home();
           abort_flag = true;
            
        #else
          checkkey = Back_Main;
          // Wait for planner moves to finish!
          if (HMI_flag.home_flag) planner.synchronize();
          card.endFilePrint();
          #ifdef ACTION_ON_CANCEL
            host_action_cancel();
          #endif
          #ifdef EVENT_GCODE_SD_STOP
            Popup_Window_Home();
            queue.inject_P(PSTR(EVENT_GCODE_SD_STOP)); // For Ender 3 "G28 X Y"
          #endif
          abort_flag = true;
        #endif
      }
      else {
         
        Goto_PrintProcess(); // cancel stop
      }
    }
  }
  DWIN_UpdateLCD();
}

inline void Draw_Move_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
 // 
  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1,156,50,200,66,285,12);//轴移动
    //DWIN_Frame_AreaCopy(1, 192, 1, 271 - 38, 479 - 465, 14, 8);
    DWIN_Frame_AreaCopy(1, 0, 86, 54, 102, LBLX, MBASE(1));
    DWIN_Frame_AreaCopy(1, 56, 86, 110, 102, LBLX, MBASE(2));
    DWIN_Frame_AreaCopy(1, 115, 86, 168, 102, LBLX, MBASE(3));
    DWIN_Frame_AreaCopy(1, 170, 86, 215, 102, LBLX, MBASE(4));
  }
  else {
    #ifdef USE_STRING_HEADINGS
       Draw_Title("Move"); // TODO: GET_TEXT_F
    #else
       DWIN_Frame_AreaCopy(1, 360, 50, 420, 64, 275, 12); // "Move"
    #endif
    draw_move_en(MBASE(1)); say_x(33 + 6, MBASE(1));                    // "Move X"
    draw_move_en(MBASE(2)); say_y(33 + 6, MBASE(2));                    // "Move Y"
    draw_move_en(MBASE(3)); say_z(33 + 6 ,MBASE(3));                    // "Move Z"
     DWIN_Frame_AreaCopy(1, 150, 184, 210, 200, LBLX, MBASE(4)); // "Extruder"
  }

  Draw_Back_First(select_axis.now == 0);
  if (select_axis.now) Draw_Menu_Cursor(select_axis.now);

  LOOP_L_N(i, MROWS) Draw_Menu_Line(i + 1, ICON_MoveX + i);
}
void Draw_matic_Menu(){
      Clear_Main_Window();
      Draw_Steady_State();
      if(HMI_flag.language_flag){
        if(checkkey==Feeding)
        {
         DWIN_Frame_AreaCopy(1,220,122,280,138,285, 12);
        }
        else
        {
           DWIN_Frame_AreaCopy(1,282,122,340,138,285, 12);
        }
          DWIN_ICON_Show(ICON,25,160,225);                 //返回箭头
          DWIN_Frame_AreaCopy(1, 124, 50, 150,64, 170,225);  //返回
      }
      else
      {
        if(checkkey==Feeding)
        {
            DWIN_Frame_AreaCopy(1,0,205,115,220,250, 12);
        }
        else
        {
          DWIN_Frame_AreaCopy(1,120,205,255,220,240,12);
        }
        
        DWIN_ICON_Show(ICON,25,155,225);                 //返回箭头
        DWIN_Frame_AreaCopy(1, 340, 166, 400,180, LBLX, MBASE(4)-10);
      }
  DWIN_Draw_Line(0X0000,140,210,450,210);
  DWIN_Draw_Rectangle(1, Red_Color, 140, 210,150,250);
  DWIN_Draw_Line(0X0000,140,250,450,250); 
}

void Draw_NO_Bltouch_Menu(void){
      Clear_Main_Window();
      Draw_Steady_State();
      if(HMI_flag.language_flag)
      {
          //DWIN_Frame_AreaCopy(1,12,122,100,138,285,12);
          DWIN_ICON_Show(ICON1,28,250,12);
          DWIN_ICON_Show(ICON1,34,140,50);
          DWIN_ICON_Show(ICON,25,160,225);                 //返回箭头
          DWIN_Frame_AreaCopy(1, 124, 50, 150,64, 170,225);  //返回
      }
      else
      {
        //DWIN_Frame_AreaCopy(1,220,122,280,138,285, 12);
        DWIN_ICON_Show(ICON1,29,250,12);
        DWIN_ICON_Show(ICON1,34,140,50);
        DWIN_ICON_Show(ICON,25,160,225);                 //返回箭头
        DWIN_Frame_AreaCopy(1, 124, 50, 150,64, 170,225);  //返回
      }
      
      DWIN_Draw_Line(0X0000,140,210,450,210);
      DWIN_Draw_Rectangle(1, Red_Color, 140, 210,150,250);
      DWIN_Draw_Line(0X0000,140,250,450,250); 
}

void Draw_Bltouch_Menu(void)
{
      Clear_Main_Window();
      Draw_Steady_State();
      if(HMI_flag.language_flag)
      {
          DWIN_ICON_Show(ICON1,28,250,12);                 //title
          DWIN_ICON_Show(ICON,25,160,225);                 //返回箭头
          DWIN_ICON_Show(ICON1,30,140,50);                 //decting
          DWIN_ICON_Show(ICON1,27,240,80);                 //16格 
          DWIN_Frame_AreaCopy(1, 124, 50, 150,64, 170,225);  //返回       
      }
      else
      {
          DWIN_ICON_Show(ICON1,29,250,12);                 //title
          DWIN_ICON_Show(ICON,25,160,225);                 //返回箭头
          DWIN_ICON_Show(ICON1,27,240,80);                 //16格
          DWIN_ICON_Show(ICON1,31,140,50);                 
          DWIN_Frame_AreaCopy(1, 124, 50, 150,64, 170,225);  //返回 
      }
      
      DWIN_Draw_Line(0X0000,140,210,450,210);
      DWIN_Draw_Rectangle(1, Red_Color, 140, 210,150,250);
      DWIN_Draw_Line(0X0000,140,250,450,250);      
}
/* Prepare */
void HMI_Control(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_control.inc(9)) {
      if (select_control.now > MROWS && select_control.now > index_control) {
        index_control = select_control.now;
        // Scroll up and draw a blank bottom line
        Scroll_Menu(DWIN_SCROLL_UP);
        //Draw_Menu_Icon(MROWS, ICON_Axis + select_prepare.now - 1);
        // Draw "More" icon for sub-menus
     
        if (index_control == 5) Prepare_Item_PLA(MROWS);
        else if (index_control == 6) Prepare_Item_ABS (MROWS);
        else if (index_control == 7) Prepare_Item_feeding(MROWS);
        else if (index_control == 8) Prepare_Item_Desizing(MROWS);
        else if (index_control == 9) Prepare_Item_Cool(MROWS);
      }
      else {
        Move_Highlight(1, select_control.now + MROWS - index_control);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_control.dec()) {
      if (select_control.now < index_control - MROWS) {
        index_control--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
        if (index_control == MROWS)
          Draw_Back_First();
        else
          Draw_Menu_Line(0, ICON_Axis + select_control.now - 1);
          if(index_control==5) Prepare_Item_Move(0);
          else if(index_control==6) Prepare_Item_Disable(0);
          else if(index_control==7) Prepare_Item_Home(0);
          else if(index_control == 8)  Prepare_Item_Offset(0);
      }
      else {
        Move_Highlight(-1, select_control.now + MROWS - index_control);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_control.now) {
      case 0: // back
        select_page.set(1);
        Goto_MainMenu();
        break;
      case 1: // axis move
        checkkey = AxisMove;
        select_axis.reset();
        Draw_Move_Menu();
        last_E_scale= current_position.e;
        HMI_ValueStruct.Move_E_scale = 0;
       // current_position.e = HMI_ValueStruct.Move_E_scale = 0;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(1), current_position[X_AXIS] * MINUNITMULT);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(2), current_position[Y_AXIS] * MINUNITMULT);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(3), current_position[Z_AXIS] * MINUNITMULT);
        show_plus_or_minus(font8x16,Black, Background_black, 3, 1, 400, MBASE(4), 0 * MINUNITMULT);
        break;
      case 2: // close motion
        queue.inject_P(PSTR("M84"));
        break;
      case 3: // homing
        checkkey = Last_Control;
        index_control = MROWS;
        queue.inject_P(PSTR("G28")); // G28 will set home_flag
        Popup_Window_Home();
        break;
      case 4: // Z-offset
          // Apply workspace offset, making the current position 0,0,0
          queue.inject_P(PSTR("G92 X0 Y0 Z0"));
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        break;
      case 5: // PLA preheat
        thermalManager.setTargetHotend(ui.material_preset[0].hotend_temp, 0);
        thermalManager.setTargetBed(ui.material_preset[0].bed_temp);
        thermalManager.set_fan_speed(0, ui.material_preset[0].fan_speed);
        break;
      case 6: // ABS preheat
        thermalManager.setTargetHotend(ui.material_preset[1].hotend_temp, 0);
        thermalManager.setTargetBed(ui.material_preset[1].bed_temp);
        thermalManager.set_fan_speed(0, ui.material_preset[1].fan_speed);
        break;
      case 7: 
        checkkey=Feeding;
        queue.inject_P(PSTR("M140 S0"));
        queue.enqueue_now_P(PSTR("M109 S235"));
        Draw_matic_Menu();
        if(HMI_flag.language_flag)
        DWIN_ICON_Show(ICON1,8,140,50);
        else
        DWIN_ICON_Show(ICON1,5,140,50);
        break;
      case 8: 
          checkkey=Desizing;
          queue.inject_P(PSTR("M140 S0"));
          queue.enqueue_now_P(PSTR("M109 S235"));
          Draw_matic_Menu();
          if(HMI_flag.language_flag)
          DWIN_ICON_Show(ICON1,1,140,50);
          else
          {
             DWIN_ICON_Show(ICON1,3,140,50);
          }
        break;
      case 9:  // cool
            thermalManager.zero_fan_speeds();
            thermalManager.disable_all_heaters();
      break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

void Draw_Temperature_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
  if (HMI_flag.language_flag) {
    //DWIN_Frame_AreaCopy(1, 380, 2, 408,18, 285, 12);
    DWIN_Frame_AreaCopy(1,250,86,278,102,285, 12);
    DWIN_Frame_AreaCopy(1, 220, 86, 278, 102, LBLX, MBASE(1));
    DWIN_Frame_AreaCopy(1, 280, 86, 338, 102, LBLX, MBASE(2));
    DWIN_Frame_AreaCopy(1, 340, 86, 400, 102,LBLX,MBASE(3));
    DWIN_Frame_AreaCopy(1,372,50,456,66,LBLX,MBASE(4));
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Temperature"); // TODO: GET_TEXT_F
    #else
     DWIN_Frame_AreaCopy(1,310,82,405,100,255,12);
    #endif
     // DWIN_Frame_AreaCopy(1,200,100,280,120,LBLX, MBASE(1));
      DWIN_Frame_AreaCopy(1,300,100,350,120,LBLX, MBASE(1));
      DWIN_Frame_AreaCopy(1,310,82,405,100,LBLX+50, MBASE(1));

      DWIN_Frame_AreaCopy(1,352,100,378,120,LBLX, MBASE(2));
      DWIN_Frame_AreaCopy(1,310,82,405,100,LBLX+26, MBASE(2));
      DWIN_Frame_AreaCopy(1,0,120,76,140,LBLX, MBASE(3));

      DWIN_Frame_AreaCopy(1, 420, 18, 474, 34, LBLX, MBASE(4)); // "Preheat"
      DWIN_Frame_AreaCopy(1, 0, 48, 27, 64, LBLX + 49 + 12, MBASE(4)); // "ABS"
      DWIN_Frame_AreaCopy(1,220,48,270,64,LBLX +30 +49 + 12, MBASE(4));

  }

   Draw_Back_First(select_temp.now == 0);
  if (select_temp.now) Draw_Menu_Cursor(select_temp.now);
  LOOP_L_N(i, 4) Draw_Menu_Line(i + 1, ICON_SetEndTemp + i);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black,White , 3, 400, MBASE(1), thermalManager.temp_hotend[0].target);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(2), thermalManager.temp_bed.target);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, White, 3, 400, MBASE(3), thermalManager.fan_speed[0]);
  Draw_More_Right_Icon(4);
}

/* Control */
void HMI_Prepare(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    #define CONTROL_ITEMS (6)
    if (select_prepare.inc(CONTROL_ITEMS)) {
      if (select_prepare.now > MROWS && select_prepare.now > index_prepare) {
        index_prepare = select_prepare.now;
        Scroll_Menu(DWIN_SCROLL_UP);
        if (index_prepare == 5) {
             Prepare_Item_Lang(MROWS);
        }
        if(index_prepare==6){
            Draw_Resume_Menu(MROWS);
        }
      }
      else {
        Move_Highlight(1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_prepare.dec()) {
      if (select_prepare.now < index_prepare - MROWS) {
        index_prepare--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
        if (index_prepare == 4)
        {
            Draw_Back_First();
        }
        if(index_prepare==5)
        {
            Draw_temp_Menu(0);
        }
      }
      else {
        Move_Highlight(-1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_prepare.now) {
      case 0: // back
        select_page.set(2);
        Goto_MainMenu();
        break;
      case 1: // temperature
        checkkey = TemperatureID;
        HMI_ValueStruct.show_mode = -1;
        select_temp.reset();
       // index_temperature=MROWS;
        Draw_Temperature_Menu();

        break;
      case 2: // motion
        checkkey = Motion;
        select_motion.reset();
        Draw_Motion_Menu();
        break;
      case 3:
          if(filament_flag==false)
          {
            DWIN_ICON_Show(ICON2,18,400,MBASE(3 + MROWS - index_prepare));
            filament_flag=true;
            set_filament();
          }
          else
          {
             DWIN_ICON_Show(ICON2,17,400,MBASE(3 + MROWS - index_prepare));
             filament_flag=false;
             set_no_filament(); 
          }
      break;
      case 4: // read EEPROM
        if (settings.load()) {
     
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else {buzzer.tone(20, 440);}
        break;
      // case 4: //
      //     checkkey = Homeoffset;
      //     HMI_ValueStruct.show_mode = -4;
      //     HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
      //     show_plus_or_minus(font8x16,White,Select_Color, 2, 2, 400, MBASE(4 + MROWS - index_prepare), HMI_ValueStruct.offset_value);
      //     EncoderRate.encoderRateEnabled = 1;
      //     break;
      case 5: //lang
          HMI_flag.language_flag = !HMI_flag.language_flag;
          if (HMI_flag.language_flag) {
            set_chinese_to_eeprom();
            DWIN_JPG_CacheTo1(Language_Chinese);
          }
          else {
            set_english_to_eeprom();
           DWIN_JPG_CacheTo1(Language_English);
          }
          select_page.set(0);
          Goto_MainMenu();
          break;
      case 6: // reset
        // checkkey = Info;
        // Draw_Info_Menu();
         settings.reset();
          HMI_ValueStruct.show_mode = -4;
          BABY_Z_VAR=0;
          HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
        #if HAS_BUZZER
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        #endif
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Leveling */
void HMI_Leveling(void) {
  Popup_Window_Leveling();
  DWIN_UpdateLCD();
  queue.inject_P(PSTR("G28O\nG29"));
}

/* Axis Move */
void HMI_AxisMove(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  #if HAS_HOTEND
    // popup window resume
    if (HMI_flag.ETempTooLow_flag) {
      if (encoder_diffState == ENCODER_DIFF_ENTER) {
        HMI_flag.ETempTooLow_flag = 0;
        Draw_Move_Menu();
        current_position.e = HMI_ValueStruct.Move_E_scale = 0;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, White, 3, 1, 400, MBASE(1), HMI_ValueStruct.Move_X_scale);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, White, 3, 1, 400, MBASE(2), HMI_ValueStruct.Move_Y_scale);
        DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, White, 3, 1, 400, MBASE(3), HMI_ValueStruct.Move_Z_scale);
        show_plus_or_minus(font8x16, Black,White, 3, 1, 400, MBASE(4), HMI_ValueStruct.Move_E_scale);
        DWIN_UpdateLCD();
      }
      return;
    }
  #endif

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_axis.inc(4)) Move_Highlight(1, select_axis.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_axis.dec()) Move_Highlight(-1, select_axis.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_axis.now) {
      case 0: // back
        checkkey = Control;
        select_control.set(1);
        index_control = MROWS;
        Draw_Control_Menu();
        break;
      case 1: // X axis move
        checkkey = Move_X;
        HMI_ValueStruct.Move_X_scale = current_position[X_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(1), HMI_ValueStruct.Move_X_scale);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 2: // Y axis move
        checkkey = Move_Y;
        HMI_ValueStruct.Move_Y_scale = current_position[Y_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(2), HMI_ValueStruct.Move_Y_scale);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 3: // Z axis move
        checkkey = Move_Z;
        HMI_ValueStruct.Move_Z_scale = current_position[Z_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(3), HMI_ValueStruct.Move_Z_scale);
        EncoderRate.encoderRateEnabled = 1;
        break;
        #if HAS_HOTEND
      case 4: // Extruder
        // window tips
        #ifdef PREVENT_COLD_EXTRUSION
          if (thermalManager.temp_hotend[0].celsius < EXTRUDE_MINTEMP) {
            HMI_flag.ETempTooLow_flag = 1;
            Popup_Window_ETempTooLow();
            DWIN_UpdateLCD();
            return;
          }
        #endif
        checkkey = Extruder;
        HMI_ValueStruct.Move_E_scale = current_position.e * MINUNITMULT-last_E_scale*MINUNITMULT;
        show_plus_or_minus(font8x16, White,Select_Color, 3, 1, 400, MBASE(4), HMI_ValueStruct.Move_E_scale);
        EncoderRate.encoderRateEnabled = 1;
        break;
        #endif
      default:
      break;
    }
  }
  DWIN_UpdateLCD();
}

/* TemperatureID */
void HMI_Temperature(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_temp.inc(5))
    {
          if(select_temp.now> MROWS &&select_temp.now>index_temperature)
          {
                index_temperature=select_temp.now;
                Scroll_Menu(DWIN_SCROLL_UP);
                if(index_temperature==5)
                {
                   Draw_ABStemp_Menu(MROWS);  
                }
          }
          else
          {
            Move_Highlight(1, select_temp.now+MROWS-index_temperature);
          }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_temp.dec()){
            if(select_temp.now<index_temperature-MROWS)
            {
                index_temperature--;
                Scroll_Menu(DWIN_SCROLL_DOWN);
                if (index_temperature == 4)
                {
                    Draw_Back_First();
                }
                if(index_temperature==5)
                {
                      Draw_Extruder_Menu(0);
                }
            }
            else
            {
                  Move_Highlight(-1, select_temp.now + MROWS - index_temperature);
            }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_temp.now) {
      case 0: // back
        checkkey = Prepare;
        select_prepare.set(1);
        index_prepare = MROWS;
        Draw_Prepare_Menu();
        break;
        #if HAS_HOTEND
          case 1: // nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White,Select_Color , 3, 400, MBASE(1 + MROWS - index_temperature), thermalManager.temp_hotend[0].target);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_HEATED_BED
          case 2: // bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2 + MROWS - index_temperature), thermalManager.temp_bed.target);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_FAN
          case 3: // fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3 + MROWS - index_temperature), thermalManager.fan_speed[0]);
            
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_HOTEND
          case 4: // PLA preheat setting
            checkkey = PLAPreheat;
            select_PLA.reset();
            HMI_ValueStruct.show_mode = -2;

            Clear_Main_Window();
            Draw_Steady_State();
            if (HMI_flag.language_flag) {
              DWIN_Frame_AreaCopy(1,372,50,456,66,255,12);
              DWIN_Frame_AreaCopy(1,372,50,398,66,LBLX, MBASE(1));
              DWIN_Frame_AreaCopy(1, 220, 86, 278, 102, LBLX+28, MBASE(1));
              DWIN_Frame_AreaCopy(1,372,50,398,66,LBLX, MBASE(2));
              DWIN_Frame_AreaCopy(1, 280, 86, 338, 102, LBLX+28, MBASE(2));
              DWIN_Frame_AreaCopy(1,372,50,398,66,LBLX, MBASE(3));
              DWIN_Frame_AreaCopy(1, 340, 86, 400, 102,LBLX+28,MBASE(3));
              DWIN_Frame_AreaCopy(1,212,68,270,84,LBLX, MBASE(4));
            }
            else {
              #ifdef USE_STRING_HEADINGS
                Draw_Title("PLA Settings"); // TODO: GET_TEXT_F
              #else
              {
                DWIN_Frame_AreaCopy(1,0,46,26,66,260,12);
                DWIN_Frame_AreaCopy(1,220,46,270,64,260+22+5,12);
              }  //DWIN_Frame_AreaCopy(1, 56, 16, 271 - 130, 479 - 450 - 1, 14, 8);
              #endif
                DWIN_Frame_AreaCopy(1,0,46,26,66,LBLX, MBASE(1));
                DWIN_Frame_AreaCopy(1,300,100,350,120,LBLX+30, MBASE(1));
                DWIN_Frame_AreaCopy(1,310,82,405,100,LBLX+50+30, MBASE(1));

                 DWIN_Frame_AreaCopy(1,0,46,26,66,LBLX, MBASE(2));
                DWIN_Frame_AreaCopy(1,352,100,378,120,LBLX+30, MBASE(2));
                DWIN_Frame_AreaCopy(1,310,82,405,100,LBLX+26+30, MBASE(2));

                 DWIN_Frame_AreaCopy(1,0,46,26,66,LBLX, MBASE(3));
                DWIN_Frame_AreaCopy(1,0,118,76,140,LBLX+30, MBASE(3));

                DWIN_Frame_AreaCopy(1,0,165,160,185,LBLX, MBASE(4));

            }

            Draw_Back_First();

            Draw_Menu_Line(1, ICON_SetEndTemp);
            Draw_Menu_Line(2, ICON_SetBedTemp);
            Draw_Menu_Line(3, ICON_FanSpeed);
            Draw_Menu_Line(4, ICON_WriteEEPROM);

            DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(1), ui.material_preset[0].hotend_temp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2), ui.material_preset[0].bed_temp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3), ui.material_preset[0].fan_speed);

            break;

          case 5: // ABS preheat setting
            checkkey = ABSPreheat;
            select_ABS.reset();
            HMI_ValueStruct.show_mode = -3;
            Clear_Main_Window();
            Draw_Steady_State();
            if (HMI_flag.language_flag) {
               DWIN_Frame_AreaCopy(1,220,20,306,36,255,12);
              DWIN_Frame_AreaCopy(1,220,20,246,36,LBLX, MBASE(1));
              DWIN_Frame_AreaCopy(1, 220, 86, 278, 102, LBLX+28, MBASE(1));
              DWIN_Frame_AreaCopy(1,220,20,246,36,LBLX, MBASE(2));
              DWIN_Frame_AreaCopy(1, 280, 86, 338, 102, LBLX+28, MBASE(2));
              DWIN_Frame_AreaCopy(1,220,20,246,36,LBLX, MBASE(3));
              DWIN_Frame_AreaCopy(1, 340, 86, 400, 102,LBLX+28,MBASE(3));
              DWIN_Frame_AreaCopy(1,212,68,270,84,LBLX, MBASE(4));
            }
            else {
              #ifdef USE_STRING_HEADINGS
                Draw_Title("ABS Settings"); // TODO: GET_TEXT_F
              #else
                //DWIN_Frame_AreaCopy(1, 56, 16, 271 - 130, 479 - 450 - 1, 14, 8);
                  DWIN_Frame_AreaCopy(1, 28, 46, 55, 64, 260, 12);
                  DWIN_Frame_AreaCopy(1,220,46,270,64,260+27+5,12);
              #endif
          
                DWIN_Frame_AreaCopy(1, 28, 46, 55, 64, LBLX, MBASE(1)); // "ABS"
                DWIN_Frame_AreaCopy(1,300,100,350,120,LBLX+30, MBASE(1));
                DWIN_Frame_AreaCopy(1,310,82,405,100,LBLX+50+30, MBASE(1));

                DWIN_Frame_AreaCopy(1, 28, 46, 55, 64, LBLX, MBASE(2)); // "ABS"
                DWIN_Frame_AreaCopy(1,352,100,378,120,LBLX+30, MBASE(2));
                DWIN_Frame_AreaCopy(1,310,82,405,100,LBLX+26+30, MBASE(2));

                DWIN_Frame_AreaCopy(1, 28, 46, 55, 64, LBLX, MBASE(3)); // "ABS"
                DWIN_Frame_AreaCopy(1,0,118,76,140,LBLX+30, MBASE(3));

               
                DWIN_Frame_AreaCopy(1,0,165,160,185,LBLX, MBASE(4));
                 DWIN_Frame_AreaCopy(1, 28, 46, 55, 64, LBLX+40, MBASE(4)); // "ABS"
            }

            Draw_Back_First();

            Draw_Menu_Line(1, ICON_SetEndTemp);
            Draw_Menu_Line(2, ICON_SetBedTemp);
            Draw_Menu_Line(3, ICON_FanSpeed);
            Draw_Menu_Line(4, ICON_WriteEEPROM);

            DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(1), ui.material_preset[1].hotend_temp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(2), ui.material_preset[1].bed_temp);
            DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(3), ui.material_preset[1].fan_speed);

            break;
        #endif // if HAS_HOTEND
    }
  }
  DWIN_UpdateLCD();
}

inline void Draw_Max_Speed_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
    //DWIN_Frame_AreaCopy(1, 406, 86, 460, 102, LBLX, MBASE(1));     // max speed

  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 406, 86, 462, 102, 270, 12);     // max speed
    DWIN_Frame_AreaCopy(1, 406, 86, 462, 102, LBLX, MBASE(1));     // max speed
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+57, MBASE(1),"X");
    DWIN_Frame_AreaCopy(1, 406, 86, 462, 102, LBLX, MBASE(2));     // max speed
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+57, MBASE(2),"Y");
    DWIN_Frame_AreaCopy(1, 406, 86, 462, 102, LBLX, MBASE(3));     // max speed
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+57, MBASE(3),"Z");
    DWIN_Frame_AreaCopy(1, 406, 86, 462, 102, LBLX, MBASE(4));     // max speed
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+57, MBASE(4),"E");

  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Max Speed"); // TODO: GET_TEXT_F
    #else
    DWIN_Frame_AreaCopy(1, 270, 120, 300, 140, 265, 12);   // "Max"                                              // "Max speed"
    DWIN_Frame_AreaCopy(1, 200, 120, 244, 140, 265+32+5, 12); // "Speed"                            
    #endif

    draw_max_en(MBASE(1));                                                    // "Max"
    DWIN_Frame_AreaCopy(1, 200, 120, 254, 140, LBLX + 30 + 5, MBASE(1)); // "Speed X"

    draw_max_en(MBASE(2));                                                    // "Max"
    draw_speed_en(30 + 5, MBASE(2));                                          // "Speed"
    say_y(30 + 45 + 6, MBASE(2));                                             // "Y"

    draw_max_en(MBASE(3));                                                    // "Max"
    draw_speed_en(30 + 5, MBASE(3));                                          // "Speed"
    say_z(30 + 45 + 6, MBASE(3));                                             // "Z"

    draw_max_en(MBASE(4));                                                    // "Max"
    draw_speed_en(30 + 5, MBASE(4));                                          // "Speed"
    say_e(30 + 45 + 6, MBASE(4));                                             // "E"
  }

  Draw_Back_First();
  LOOP_L_N(i, 4) Draw_Menu_Line(i + 1, ICON_MaxSpeedX + i);

  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(1), planner.settings.max_feedrate_mm_s[X_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(2), planner.settings.max_feedrate_mm_s[Y_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(3), planner.settings.max_feedrate_mm_s[Z_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(4), planner.settings.max_feedrate_mm_s[E_AXIS]);
}

inline void Draw_Max_Accel_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();

  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 406, 86, 436, 102, 260, 12);     // max...
    DWIN_Frame_AreaCopy(1, 62, 104, 110, 120, 260 + 27, 12); // ...acceleration


    DWIN_Frame_AreaCopy(1, 406, 86, 436, 102, LBLX, MBASE(1));     // max...
    DWIN_Frame_AreaCopy(1, 62, 104, 110, 120, LBLX + 27, MBASE(1)); // ...acceleration
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+75, MBASE(1),"X");
    DWIN_Frame_AreaCopy(1, 406, 86, 436, 102, LBLX, MBASE(2));     // max...
    DWIN_Frame_AreaCopy(1, 62, 104, 110, 120, LBLX + 27, MBASE(2)); // ...acceleration
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+75, MBASE(2),"Y");
    DWIN_Frame_AreaCopy(1, 406, 86, 436, 102, LBLX, MBASE(3));     // max...
    DWIN_Frame_AreaCopy(1, 62, 104, 110, 120, LBLX + 27, MBASE(3)); // ...acceleration
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+75, MBASE(3),"Z");
    DWIN_Frame_AreaCopy(1, 406, 86, 436, 102, LBLX, MBASE(4));     // max...
    DWIN_Frame_AreaCopy(1, 62, 104, 110, 120, LBLX + 27, MBASE(4)); // ...acceleration
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+75, MBASE(4),"E");
   
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Max Acceleration"); // TODO: GET_TEXT_F
    #else
      DWIN_Frame_AreaCopy(1, 270, 120, 300, 140, 250, 12);   // "Max"
      DWIN_Frame_AreaCopy(1, 305, 120, 410, 140, 250 + 30 + 5, 12); // "Acceleration"
    #endif
    draw_max_accel_en(MBASE(1)); say_x(40 + 80 + 10, MBASE(1)); // "Max Acceleration X"
    draw_max_accel_en(MBASE(2)); say_y(40 + 80 + 10, MBASE(2)); // "Max Acceleration Y"
    draw_max_accel_en(MBASE(3)); say_z(40 + 80 + 10, MBASE(3)); // "Max Acceleration Z"
    draw_max_accel_en(MBASE(4)); say_e(40 + 80 + 10, MBASE(4)); // "Max Acceleration E"
  }

  Draw_Back_First();
  LOOP_L_N(i, 4) Draw_Menu_Line(i + 1, ICON_MaxAccX + i);

  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(1), planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(2), planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(3), planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 4, 400, MBASE(4), planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
}

inline void Draw_Max_Jerk_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();
  if (HMI_flag.language_flag) {

      DWIN_Frame_AreaCopy(1, 406, 86, 432, 102, 250, 12);;     // max...
      DWIN_Frame_AreaCopy(1, 154, 122, 185, 138, 250 + 27, 12); // ...
      DWIN_Frame_AreaCopy(1, 436, 86, 462, 102,   250 + 60, 12); // ...jerk

      DWIN_Frame_AreaCopy(1, 406, 86, 432, 102, LBLX, MBASE(1));;     // max...
      DWIN_Frame_AreaCopy(1, 154, 122, 185, 138, LBLX + 27, MBASE(1)); // ...
      DWIN_Frame_AreaCopy(1, 436, 86, 462, 102,   LBLX + 60, MBASE(1)); // ...jerk
      DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+88, MBASE(1),"X");

      DWIN_Frame_AreaCopy(1, 406, 86, 432, 102, LBLX, MBASE(2));;     // max...
      DWIN_Frame_AreaCopy(1, 154, 122, 185, 138, LBLX + 27, MBASE(2)); // ...
      DWIN_Frame_AreaCopy(1, 436, 86, 462, 102,   LBLX + 60, MBASE(2)); // ...jerk
      DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+88, MBASE(2),"Y");

      DWIN_Frame_AreaCopy(1, 406, 86, 432, 102, LBLX, MBASE(3));;     // max...
      DWIN_Frame_AreaCopy(1, 154, 122, 185, 138, LBLX + 27, MBASE(3)); // ...
      DWIN_Frame_AreaCopy(1, 436, 86, 462, 102,   LBLX + 60, MBASE(3)); // ...jerk
      DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+88, MBASE(3),"Z");

      DWIN_Frame_AreaCopy(1, 406, 86, 432, 102, LBLX, MBASE(4));;     // max...
      DWIN_Frame_AreaCopy(1, 154, 122, 185, 138, LBLX + 27, MBASE(4)); // ...
      DWIN_Frame_AreaCopy(1, 436, 86, 462, 102,   LBLX + 60, MBASE(4)); // ...jerk
      DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+88, MBASE(4),"E");
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Max Corner"); // TODO: GET_TEXT_F
    #else
     // DWIN_Frame_AreaCopy(1, 144, 16, 271 - 82, 479 - 453, 14, 8);
       DWIN_Frame_AreaCopy(1, 270, 120, 300, 140, 260, 12);   // "Max"
      DWIN_Frame_AreaCopy(1, 80, 120, 128, 140, 260 + 30 + 5, 12); // "Corner"
    #endif

    draw_max_en(MBASE(1));            // "Max"
    draw_corner_en(MBASE(1));         // "Corner"
    draw_speed_en(70 + 16, MBASE(1));  // "Speed"
    say_x(110 + 22, MBASE(1));         // "X"

    draw_max_en(MBASE(2));            // "Max"
    draw_corner_en(MBASE(2));         // "Corner"
    draw_speed_en(70 + 16, MBASE(2));  // "Speed"
    say_y(110 + 22, MBASE(2));         // "Y"

    draw_max_en(MBASE(3));            // "Max"
    draw_corner_en(MBASE(3));         // "Corner"
    draw_speed_en(70 + 16, MBASE(3));  // "Speed"
    say_z(110 + 22, MBASE(3));         // "Z"

    draw_max_en(MBASE(4));            // "Max"
    draw_corner_en(MBASE(4));         // "Corner"
    draw_speed_en(70 + 16, MBASE(4));  // "Speed"
    say_e(110 + 22, MBASE(4));         // "E"
  }

  Draw_Back_First();
  LOOP_L_N(i, 4) Draw_Menu_Line(i + 1, ICON_MaxSpeedCornerX + i);

  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(1), planner.max_jerk[X_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(2), planner.max_jerk[Y_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(3), planner.max_jerk[Z_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(4), planner.max_jerk[E_AXIS] * MINUNITMULT);
}

inline void Draw_Steps_Menu() {
  Clear_Main_Window();
  Draw_Steady_State();

  if (HMI_flag.language_flag) {
    DWIN_Frame_AreaCopy(1, 200, 104, 246, 120, 285, 12); // flow ratio

    DWIN_Frame_AreaCopy(1, 200, 104, 246, 120, LBLX, MBASE(1)); // flow ratio
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+48, MBASE(1),"X");
    DWIN_Frame_AreaCopy(1, 200, 104, 246, 120, LBLX, MBASE(2)); // flow ratio
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+48, MBASE(2),"Y");
    DWIN_Frame_AreaCopy(1, 200, 104, 246, 120, LBLX, MBASE(3)); // flow ratio
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+48, MBASE(3),"Z");
    DWIN_Frame_AreaCopy(1, 200, 104, 246, 120, LBLX, MBASE(4)); // flow ratio
    DWIN_Draw_String(false,true,font8x16,Black,White,LBLX+48, MBASE(4),"E");
  }
  else {
    #ifdef USE_STRING_HEADINGS
      Draw_Title("Steps-per-mm"); // TODO: GET_TEXT_F
    #else
      //DWIN_Frame_AreaCopy(1, 144, 16, 271 - 82, 479 - 453, 14, 8);
       DWIN_Frame_AreaCopy(1, 0, 145, 105, 165, 255, 12);   // "Steps-per-mm"
    #endif
    draw_steps_per_mm(MBASE(1)); say_x(100 + 5, MBASE(1)+4); // "Steps-per-mm X"
    draw_steps_per_mm(MBASE(2)); say_y(100 + 5, MBASE(2)+4); // "Y"
    draw_steps_per_mm(MBASE(3)); say_z(100 + 5, MBASE(3)+4); // "Z"
    draw_steps_per_mm(MBASE(4)); say_e(100 + 5, MBASE(4)+4); // "E"
  }

  Draw_Back_First();
  LOOP_L_N(i, 4) Draw_Menu_Line(i + 1, ICON_StepX + i);

  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(1), planner.settings.axis_steps_per_mm[X_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(2), planner.settings.axis_steps_per_mm[Y_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(3), planner.settings.axis_steps_per_mm[Z_AXIS] * MINUNITMULT);
  DWIN_Draw_FloatValue(true, true, 0, font8x16, Black, Background_black, 3, 1, 400, MBASE(4), planner.settings.axis_steps_per_mm[E_AXIS] * MINUNITMULT);
}

/* Motion */
void HMI_Motion(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_motion.inc(4)) Move_Highlight(1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_motion.dec()) Move_Highlight(-1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_motion.now) {
      case 0: // back
          checkkey = Prepare;
          select_prepare.set(2);
          index_prepare= MROWS;
          Draw_Prepare_Menu();
        break;
      case 1: // max speed
        checkkey = MaxSpeed;
        select_speed.reset();
        Draw_Max_Speed_Menu();
        break;
      case 2: // max acceleration
        checkkey = MaxAcceleration;
        select_acc.reset();
        Draw_Max_Accel_Menu();
        break;
      case 3: // max corner speed
        checkkey = MaxCorner;
        select_corner.reset();
        Draw_Max_Jerk_Menu();
        break;
      case 4: // transmission ratio
        checkkey = Step;
        select_step.reset();
        Draw_Steps_Menu();
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Info */
void HMI_Info(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_ENTER) {
    #if 0
      checkkey = Control;
      select_control.set(CONTROL_ITEMS);
      Draw_Control_Menu();
    #else
      select_page.set(3);
      Goto_MainMenu();
    #endif
  }
  DWIN_UpdateLCD();
}

/* Tune */
void HMI_Tune(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_tune.inc(5)) {
      if (select_tune.now > MROWS && select_tune.now > index_tune) {
        index_tune = select_tune.now;
        Scroll_Menu(DWIN_SCROLL_UP);
          if(index_tune==5)
          {
              Draw_Zoffset_Menu(MROWS);
              show_plus_or_minus(font8x16,Black, Background_black, 2, 2, 400, MBASE(5 + MROWS - index_tune), HMI_ValueStruct.offset_value);
          }
          // if(index_tune==6)
          // {
          //    //Draw_feeding_Menu(MROWS);
          // }
      }
      else {
        Move_Highlight(1, select_tune.now + MROWS - index_tune);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_tune.dec()) {
      if (select_tune.now < index_tune - MROWS) {
        index_tune--;
        Scroll_Menu(DWIN_SCROLL_DOWN);
        if (index_tune == MROWS)
          Draw_Back_First();
        else
        {
              Draw_Menu_Line(0, ICON_Axis + select_tune.now - 1);
              if(index_tune==4)
              {
                  if(HMI_flag.language_flag==1)
                  {
                    DWIN_Frame_AreaCopy(1, 220, 86, 280, 102, LBLX, MBASE(0));
                  }
                  else
                  {
                    DWIN_Frame_AreaCopy(1, 220, 86, 280, 102, LBLX, MBASE(0));
                  }
              }
              if(index_tune==5)
              {
                    if(HMI_flag.language_flag==1)
                    {
                          DWIN_Frame_AreaCopy(1, 0, 122, 60, 138, LBLX, MBASE(0));
                    }
                    else
                    {
                          DWIN_Frame_AreaCopy(1, 0, 122, 60, 138, LBLX, MBASE(0));
                    }
                  HMI_ValueStruct.print_speed = feedrate_percentage;
                  DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 400, MBASE(1 + MROWS - index_tune), feedrate_percentage);
              }
          }
      }
      else {
        Move_Highlight(-1, select_tune.now + MROWS - index_tune);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_tune.now) {
      case 0: { // Back
        select_print.set(0);
        Goto_PrintProcess();
      }
      break;
      case 1: // Print speed
        checkkey = PrintSpeed;
        HMI_ValueStruct.print_speed = feedrate_percentage;
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(1 + MROWS - index_tune), feedrate_percentage);
        EncoderRate.encoderRateEnabled = 1;
        break;
        #if HAS_HOTEND
          case 2: // Nozzle temp
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = thermalManager.temp_hotend[0].target;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2 + MROWS - index_tune), thermalManager.temp_hotend[0].target);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_HEATED_BED
          case 3: // Bed temp
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = thermalManager.temp_bed.target;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3 + MROWS - index_tune), thermalManager.temp_bed.target);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_FAN
          case 4: // Fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = thermalManager.fan_speed[0];
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(4 + MROWS - index_tune), thermalManager.fan_speed[0]);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
      case 5: // Z-offset
        checkkey = Homeoffset;
        HMI_ValueStruct.offset_value = BABY_Z_VAR * 100;
        show_plus_or_minus(font8x16,White,Select_Color, 2, 2, 400, MBASE(5 + MROWS - index_tune), HMI_ValueStruct.offset_value);
        EncoderRate.encoderRateEnabled = 1;
        break;
      // case 6: // Language
   
      //   break;

      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* PLA Preheat */
void HMI_PLAPreheatSetting(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_PLA.inc(4)) Move_Highlight(1, select_PLA.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_PLA.dec()) Move_Highlight(-1, select_PLA.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_PLA.now) {
      case 0: // back
        checkkey = TemperatureID;
        //select_temp.now = 4;
        select_temp.reset();
        HMI_ValueStruct.show_mode = -1;
        index_temperature = MROWS;
        select_temp.set(0);
        Draw_Temperature_Menu();
        break;
        #if HAS_HOTEND
          case 1: // set nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = ui.material_preset[0].hotend_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(1), ui.material_preset[0].hotend_temp);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_HEATED_BED
          case 2: // set bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = ui.material_preset[0].bed_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2), ui.material_preset[0].bed_temp);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_FAN
          case 3: // set fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = ui.material_preset[0].fan_speed;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3), ui.material_preset[0].fan_speed);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
      case 4: // save PLA configuration
        if (settings.save()) {
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else
          buzzer.tone(20, 440);
        break;
      default: break;
    }
  }
  DWIN_UpdateLCD();
}

/* ABS Preheat */
void HMI_ABSPreheatSetting(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_ABS.inc(4)) Move_Highlight(1, select_ABS.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_ABS.dec()) Move_Highlight(-1, select_ABS.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_ABS.now) {
      case 0: // back
        checkkey = TemperatureID;
        select_temp.reset();
        select_temp.set(0);
        index_temperature = MROWS;
        HMI_ValueStruct.show_mode = -1;
        Draw_Temperature_Menu();
        break;
        #if HAS_HOTEND
          case 1: // set nozzle temperature
            checkkey = ETemp;
            HMI_ValueStruct.E_Temp = ui.material_preset[1].hotend_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(1), ui.material_preset[1].hotend_temp);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_HEATED_BED
          case 2: // set bed temperature
            checkkey = BedTemp;
            HMI_ValueStruct.Bed_Temp = ui.material_preset[1].bed_temp;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(2), ui.material_preset[1].bed_temp);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
        #if HAS_FAN
          case 3: // set fan speed
            checkkey = FanSpeed;
            HMI_ValueStruct.Fan_speed = ui.material_preset[1].fan_speed;
            DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 3, 400, MBASE(3), ui.material_preset[1].fan_speed);
            EncoderRate.encoderRateEnabled = 1;
            break;
        #endif
      case 4: // save PLA configuration
        if (settings.save()) {
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else
          buzzer.tone(20, 440);
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Max Speed */
void HMI_MaxSpeed(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_speed.inc(5)) 
    {
       if (select_speed.now > MROWS && select_speed.now > index_speed) {
      index_speed = select_speed.now;
        Scroll_Menu(DWIN_SCROLL_UP);
        if (HMI_flag.language_flag) 
         DWIN_Frame_AreaCopy(1, 214, 68, 272, 86,LBLX, MBASE(4)); // save ABS configuration
        else
        DWIN_Frame_AreaCopy(1, 0, 100, 140, 120, LBLX, MBASE(4)); // save PLA configuration
        DWIN_Draw_Line(Line_Color, 160, 40 + 5 * MLINE, 450, 40 + 5 * MLINE);
    }
    else
    {
        Move_Highlight(1, select_speed.now + MROWS - index_speed);
    }
  }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_speed.dec()) 
    {
       if (select_speed.now < index_speed - MROWS) {
      //Move_Highlight(-1, select_speed.now);
           index_speed--;
           Scroll_Menu(DWIN_SCROLL_DOWN);
          if (index_speed == MROWS) Draw_Back_First();
       }
      else
      {
          Move_Highlight(-1, select_speed.now + MROWS - index_speed);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_speed.now) {
      case 0: // back
        checkkey = Motion;
        select_motion.now = 1;
        Draw_Motion_Menu();

        break;
      case 1: // max Speed X
        checkkey                      = MaxSpeed_value;
        HMI_flag.feedspeed_flag       = X_AXIS;
        HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[X_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(1 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
        EncoderRate.encoderRateEnabled = 1;                                          
        break;
      case 2: // max Speed Y
        checkkey                      = MaxSpeed_value;
        HMI_flag.feedspeed_flag       = Y_AXIS;
        HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[Y_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(2 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 3: // max Speed Z
        checkkey                      = MaxSpeed_value;
        HMI_flag.feedspeed_flag       = Z_AXIS;
        HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[Z_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(3 + MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 4: // max Speed E
        checkkey                      = MaxSpeed_value;
        HMI_flag.feedspeed_flag       = E_AXIS;
        HMI_ValueStruct.Max_Feedspeed = planner.settings.max_feedrate_mm_s[E_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(4+ MROWS - index_speed), HMI_ValueStruct.Max_Feedspeed);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 5:
          if (settings.save()) {
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else
          buzzer.tone(20, 440);
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Max Acceleration */
void HMI_MaxAcceleration(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_acc.inc(5)) 
    {
       if (select_acc.now > MROWS && select_acc.now > index_acc) {
      index_acc = select_acc.now;
        Scroll_Menu(DWIN_SCROLL_UP);
        if (HMI_flag.language_flag) 
         DWIN_Frame_AreaCopy(1, 214, 68, 272, 86,LBLX, MBASE(4)); // save ABS configuration
        else
        DWIN_Frame_AreaCopy(1, 0, 100, 140, 120, LBLX, MBASE(4)); // save PLA configuration
        DWIN_Draw_Line(Line_Color, 160, 40 + 5 * MLINE, 450, 40 + 5 * MLINE);
    }
    else
    {
        Move_Highlight(1, select_acc.now + MROWS - index_acc);
    }
  }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_acc.dec()) 
    {
       if (select_acc.now < index_acc - MROWS) {
      //Move_Highlight(-1, select_speed.now);
           index_acc--;
           Scroll_Menu(DWIN_SCROLL_DOWN);
          if (index_acc == MROWS) Draw_Back_First();
       }
      else
      {
          Move_Highlight(-1, select_acc.now + MROWS - index_acc);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_acc.now) {
      case 0: // back
        checkkey          = Motion;
        select_motion.now = 2;
        Draw_Motion_Menu();
        break;
      case 1: // max acceleration X
        checkkey                         = MaxAcceleration_value;
        HMI_flag.acc_flag                = X_AXIS;
        HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[X_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(1 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 2: // max acceleration Y
        checkkey                         = MaxAcceleration_value;
        HMI_flag.acc_flag                = Y_AXIS;
        HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[Y_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(2 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 3: // max acceleration Z
        checkkey                         = MaxAcceleration_value;
        HMI_flag.acc_flag                = Z_AXIS;
        HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[Z_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(3 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 4: // max acceleration E
        checkkey                         = MaxAcceleration_value;
        HMI_flag.acc_flag                = E_AXIS;
        HMI_ValueStruct.Max_Acceleration = planner.settings.max_acceleration_mm_per_s2[E_AXIS];
        DWIN_Draw_IntValue(true, true, 0, font8x16, White, Select_Color, 4, 400, MBASE(4 + MROWS - index_acc), HMI_ValueStruct.Max_Acceleration);
        EncoderRate.encoderRateEnabled = 1;
        break;
            case 5:
          if (settings.save()) {
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else
          buzzer.tone(20, 440);
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Max Corner */
void HMI_MaxCorner(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_corner.inc(5)) 
    {
       if (select_corner.now > MROWS && select_corner.now > index_corner) {
      index_corner = select_corner.now;
        Scroll_Menu(DWIN_SCROLL_UP);
        if (HMI_flag.language_flag) 
         DWIN_Frame_AreaCopy(1, 214, 68, 272, 86,LBLX, MBASE(4)); // save ABS configuration
        else
        DWIN_Frame_AreaCopy(1, 0, 100, 140, 120, LBLX, MBASE(4)); // save PLA configuration
        DWIN_Draw_Line(Line_Color, 160, 40 + 5 * MLINE, 450, 40 + 5 * MLINE);
    }
    else
    {
        Move_Highlight(1, select_corner.now + MROWS - index_corner);
    }
  }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_corner.dec()) 
    {
       if (select_corner.now < index_corner - MROWS) {
           index_corner--;
           Scroll_Menu(DWIN_SCROLL_DOWN);
          if (index_corner == MROWS) Draw_Back_First();
       }
      else
      {
          Move_Highlight(-1, select_corner.now + MROWS - index_corner);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_corner.now) {
      case 0: // back
        checkkey          = Motion;
        select_motion.now = 3;
        Draw_Motion_Menu();
        break;
      case 1: // max corner X
        checkkey                   = MaxCorner_value;
        HMI_flag.corner_flag       = X_AXIS;
        HMI_ValueStruct.Max_Corner = planner.max_jerk[X_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(1 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 2: // max corner Y
        checkkey                   = MaxCorner_value;
        HMI_flag.corner_flag       = Y_AXIS;
        HMI_ValueStruct.Max_Corner = planner.max_jerk[Y_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(2 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 3: // max corner Z
        checkkey                   = MaxCorner_value;
        HMI_flag.corner_flag       = Z_AXIS;
        HMI_ValueStruct.Max_Corner = planner.max_jerk[Z_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(3+ MROWS - index_corner), HMI_ValueStruct.Max_Corner);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 4: // max corner E
        checkkey                   = MaxCorner_value;
        HMI_flag.corner_flag       = E_AXIS;
        HMI_ValueStruct.Max_Corner = planner.max_jerk[E_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(4 + MROWS - index_corner), HMI_ValueStruct.Max_Corner);
        EncoderRate.encoderRateEnabled = 1;
        break;
            case 5:
          if (settings.save()) {
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else
          buzzer.tone(20, 440);
            break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}

/* Step */
void HMI_Step(void) {
  ENCODER_DiffState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_step.inc(5)) 
    {
       if (select_step.now > MROWS && select_step.now > index_step) {
      index_step = select_step.now;
        Scroll_Menu(DWIN_SCROLL_UP);
        if (HMI_flag.language_flag) 
         DWIN_Frame_AreaCopy(1, 214, 68, 272, 86,LBLX, MBASE(4)); // save ABS configuration
        else
        DWIN_Frame_AreaCopy(1, 0, 100, 140, 120, LBLX, MBASE(4)); // save PLA configuration
        DWIN_Draw_Line(Line_Color, 160, 40 + 5 * MLINE, 450, 40 + 5 * MLINE);
    }
    else
    {
        Move_Highlight(1, select_step.now + MROWS - index_step);
    }
  }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_step.dec()) 
    {
       if (select_step.now < index_step - MROWS) {
      //Move_Highlight(-1, select_speed.now);
           index_step--;
           Scroll_Menu(DWIN_SCROLL_DOWN);
          if (index_step == MROWS) Draw_Back_First();
       }
      else
      {
          Move_Highlight(-1, select_step.now + MROWS - index_step);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_step.now) {
      case 0: // back
        checkkey = Motion;
        select_motion.now = 4;
        Draw_Motion_Menu();
        break;
      case 1: // max step X
        checkkey                 = Step_value;
        HMI_flag.step_flag       = X_AXIS;
        HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[X_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(1 + MROWS - index_step), HMI_ValueStruct.Max_Step);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 2: // max step Y
        checkkey                 = Step_value;
        HMI_flag.step_flag       = Y_AXIS;
        HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[Y_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(2 + MROWS - index_step), HMI_ValueStruct.Max_Step);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 3: // max step Z
        checkkey                 = Step_value;
        HMI_flag.step_flag       = Z_AXIS;
        HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[Z_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(3 + MROWS - index_step), HMI_ValueStruct.Max_Step);
        EncoderRate.encoderRateEnabled = 1;
        break;
      case 4: // max step E
        checkkey                 = Step_value;
        HMI_flag.step_flag       = E_AXIS;
        HMI_ValueStruct.Max_Step = planner.settings.axis_steps_per_mm[E_AXIS] * MINUNITMULT;
        DWIN_Draw_FloatValue(true, true, 0, font8x16, White, Select_Color, 3, 1, 400, MBASE(4 + MROWS - index_step), HMI_ValueStruct.Max_Step);
        EncoderRate.encoderRateEnabled = 1;
        break;
        case 5:
          if (settings.save()) {
          buzzer.tone(100, 659);
          buzzer.tone(100, 698);
        }
        else
          buzzer.tone(20, 440);
        break;
      default:
        break;
    }
  }
  DWIN_UpdateLCD();
}
void HMI_Matic(void){
      ENCODER_DiffState encoder_diffState = get_encoder_state();
      if (encoder_diffState == ENCODER_DIFF_NO) return;
      if (encoder_diffState == ENCODER_DIFF_ENTER) {
      wait_for_heatup = false; 
      thermalManager.disable_all_heaters();
      //queue.clear();
      // quickstop_stepper();
      quickstop_stepper();
      disable_all_steppers();
      print_job_timer.stop();
      checkkey = Control;
      select_control.set(7);
      Draw_Control_Menu();
      }
  DWIN_UpdateLCD();
}
void HMI_unload(void)
{
      ENCODER_DiffState encoder_diffState = get_encoder_state();
      if (encoder_diffState == ENCODER_DIFF_NO) return;
      if (encoder_diffState == ENCODER_DIFF_ENTER) {
          wait_for_heatup = false; 
          thermalManager.disable_all_heaters();
          // disable_all_steppers();
           quickstop_stepper();
          print_job_timer.stop();
          checkkey = Control;
          select_control.set(8);
          Draw_Control_Menu();
      }
      DWIN_UpdateLCD();
}

void HMI_Bltouch(void)
{
      ENCODER_DiffState encoder_diffState = get_encoder_state();
      if (encoder_diffState == ENCODER_DIFF_NO) return;
       if (encoder_diffState == ENCODER_DIFF_ENTER) {
          if(bltouch_flag==false)
          {
              checkkey = Control;
              Draw_Control_Menu();
          }
          else
          {
              if(completedlevel_flag==false)return;
              else
              {
                completedlevel_flag=false;
                checkkey = Control;
                Draw_Control_Menu();
              }
              
          }
          
       }
}
void HMI_filament(void)
{
    ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
        Goto_PrintProcess();
    }
    DWIN_UpdateLCD();
}
void HMI_filament_exist(void)
{
   ENCODER_DiffState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (encoder_diffState == ENCODER_DIFF_CW) {
      filament_exist_select = false;
        if(HMI_flag.language_flag)
        {  
            DWIN_ICON_Show(ICON2,19,124,166);
            DWIN_ICON_Show(ICON2,24,266,166); 
        }
        else
        {
            
            DWIN_ICON_Show(ICON2,21,124,166);
            DWIN_ICON_Show(ICON2,26,266,166); 
        }
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW)
    {
          filament_exist_select = true;
          if(HMI_flag.language_flag)
          {
              
              DWIN_ICON_Show(ICON2,20,124,166);
              DWIN_ICON_Show(ICON2,23,266,166); 
          }
          else
          {
               
               DWIN_ICON_Show(ICON2,22,124,166);
               DWIN_ICON_Show(ICON2,25,266,166); 
          }
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER)
    {
          if(filament_exist_select==true)
          {
            char cmd[40];
            cmd[0] = '\0';

            #if ENABLED(PAUSE_HEAT)
              if (tempbed) sprintf_P(cmd, PSTR("M190 S%i\n"), tempbed);
              if (temphot) sprintf_P(&cmd[strlen(cmd)], PSTR("M109 S%i\n"), temphot);
            #endif

            strcat_P(cmd, PSTR("M24"));
            queue.inject(cmd);
            HMI_ValueStruct.show_mode=0;
            thermalManager.temp_bed.target=tempbed;
            thermalManager.temp_hotend[0].target=temphot;
            Goto_PrintProcess();
          }
          else
          {
              checkkey = Back_Main; 
              card.endFilePrint();
              TERN_(POWER_LOSS_RECOVERY, recovery.cancel());
              card.flag.abort_sd_printing = true;
              Popup_Window_Home();
              abort_flag = true;
          }
          
    }
    DWIN_UpdateLCD();
}
void HMI_Init(void) {
  HMI_SDCardInit();

  lcd_select_language();

  #if ENABLED(FIX_MOUNTED_PROBE) // For "CREALITY_TOUCH" probe too?
    SET_OUTPUT(COM_PIN);
    WRITE(COM_PIN, 1);
  #endif

  delay(20);
}

void DWIN_Update(void) {
  EachMomentUpdate();   // Status update
  HMI_SDCardUpdate();   // SD card update
  DWIN_HandleScreen();  // Rotary encoder update
}

void EachMomentUpdate(void) {
  static millis_t next_rts_update_ms = 0;
  const millis_t ms = millis();
  if (PENDING(ms, next_rts_update_ms)) return;
  next_rts_update_ms = ms + DWIN_SCROLL_UPDATE_INTERVAL;

  // variable update
  //SERIAL_ECHOLNPAIR("  M413 S", int(recovery.enabled));
  update_variable();
  if (checkkey == PrintProcess) {
    // if print done
    if (HMI_flag.print_finish && !HMI_flag.confirm_flag) {
      HMI_flag.print_finish = 0;
      //HMI_flag.confirm_flag = 1;
      abort_flag            = 1;
      TERN_(POWER_LOSS_RECOVERY, recovery.cancel());
      planner.finish_and_disable();
      // show percent bar and value
      //Percentrecord = 0;
      //Draw_Print_ProgressBar();
  
      select_page.set(0);
       Goto_MainMenu();
      // show print done confirm
      // DWIN_Draw_Rectangle(1, Background_black, 0, 250,  271, 360);
      // DWIN_ICON_Show(ICON, HMI_flag.language_flag ? ICON_Confirm_C : ICON_Confirm_E, 86, 302 - 19);
    }
    else if (HMI_flag.pause_flag != printingIsPaused()) {
      // print status update
      HMI_flag.pause_flag = printingIsPaused();
      if (HMI_flag.pause_flag) ICON_Continue(); else ICON_Pause();
    }
  }
  if(checkkey==PrintProcess||checkkey==Tune)
  {
      if(filament_flag==true&& filament_recovery_flag==false)
    {
      if(READ(FIL_RUNOUT_PIN)==true && filament_state == false)
      {
          filament_state = true;
          checkkey = filament;
          filament_select=false;
          pause_action_flag = 1;
        #if ENABLED(POWER_LOSS_RECOVERY)
          if (recovery.enabled) recovery.save(true);
        #endif
        queue.inject_P(PSTR("M25"));
        //queue.enqueue_now_P(PSTR("G1 F1200 X0 Y0"));
          if(HMI_flag.language_flag)
          {
              DWIN_ICON_Show(ICON1,36,0,0);
              DWIN_ICON_Show(ICON,44,210,172);
          }
          else
          {
              DWIN_ICON_Show(ICON1,37,0,0);
              DWIN_ICON_Show(ICON,49,210,172);
          }
            
      }
      if(READ(FIL_RUNOUT_PIN)==false && filament_state == true)
      {
          filament_state = false;
          filament_select = true;
      }
    }
  }
  // pause after homing
  if (pause_action_flag && printingIsPaused() && !planner.has_blocks_queued()) {
    pause_action_flag = 0;
    if(filament_flag==false)
    {
    #if ENABLED(PAUSE_HEAT)
      tempbed = thermalManager.temp_bed.target;
      temphot = thermalManager.temp_hotend[0].target;
      thermalManager.disable_all_heaters();
      queue.inject_P(PSTR("G1 F1200 X0 Y0"));
    #endif
    }
    if(filament_flag==true&&filament_recovery_flag==false)
    {
      if(filament_select==false)
      {
          tempbed = thermalManager.temp_bed.target;
          temphot = thermalManager.temp_hotend[0].target;
          queue.inject_P(PSTR("G1 F1200 X0 Y0"));
      }
      else
      {
        tempbed = thermalManager.temp_bed.target;
        temphot = thermalManager.temp_hotend[0].target;
        thermalManager.disable_all_heaters();
        queue.inject_P(PSTR("G1 F1200 X0 Y0"));
      }
    }
  }
  if (card.isPrinting() && checkkey == PrintProcess) { // print process
    const uint8_t card_pct = card.percentDone();
    static uint8_t last_cardpercentValue = 101;
    if (last_cardpercentValue != card_pct) { // print percent
      last_cardpercentValue = card_pct;
      if (card_pct) {
        Percentrecord = card_pct;
        Draw_Print_ProgressBar();
      }
    }

    duration_t elapsed = print_job_timer.duration(); // print timer
    /* already print time */
    const uint16_t min = (elapsed.value % 3600) / 60;
    if (last_Printtime != min) { // 1 minute update
      last_Printtime = min;
      Draw_Print_ProgressElapsed();
    }
    /* remain print time */
    static millis_t next_remain_time_update = 0;
    if (elapsed.minute() > 5 && ELAPSED(ms, next_remain_time_update) && HMI_flag.heat_flag == 0) { // show after 5 min and 20s update
      //remain_time = ((elapsed.value - dwin_heat_time) * ((float)card.getFileSize() / (float)card.getIndex())) - (elapsed.value - dwin_heat_time);
      remain_time = (100-Percentrecord)* (elapsed.value - dwin_heat_time)/(Percentrecord);
      next_remain_time_update += 20 * 1000UL;
      Draw_Print_ProgressRemain();
    }
  }
  else if (abort_flag && !HMI_flag.home_flag) { // Print Stop
    abort_flag = 0;
    HMI_ValueStruct.print_speed = feedrate_percentage = 100;
    zprobe_zoffset = TERN(HAS_LEVELING, probe.offset.z, 0);

    planner.finish_and_disable();

    #if DISABLED(SD_ABORT_NO_COOLDOWN)
      thermalManager.disable_all_heaters();
    #endif

      Percentrecord = 0;
      Draw_Print_ProgressBar();

    select_page.set(0);
    Goto_MainMenu();
  }
  else if (DWIN_lcd_sd_status && recovery.dwin_flag) { // resume print before power off
    recovery.dwin_flag = false;
    recovery.load();
    if (!recovery.valid()) return recovery.purge();
    filament_recovery_flag=true;
    const uint16_t fileCnt = card.get_num_Files();
    for (uint16_t i = 0; i < fileCnt; i++) {
      // TODO: Resume print via M1000 then update the UI
      // with the active filename which can come from CardReader.
      card.getfilename_sorted(SD_ORDER(i, fileCnt));
      if (!strcmp(card.filename, &recovery.info.sd_filename[1])) { // Resume print before power failure while have the same file
        recovery_flag = 1;
        HMI_flag.select_flag = 1;
        Popup_Window_Resume();
        char * const name = card.longest_filename();
        char temp_str[strlen(name) + 1];
        make_name_without_ext(temp_str, name);
        DWIN_Draw_String(false, true, font8x16, Font_window, Red_Color, (DWIN_WIDTH - strlen(temp_str) * 9) / 2, 135, temp_str);
        DWIN_UpdateLCD();
        break;
      }
    }
    // if hasn't resumable G-code file
    if (!recovery_flag) return;
    while (recovery_flag) {
      ENCODER_DiffState encoder_diffState = Encoder_ReceiveAnalyze();
      if (encoder_diffState != ENCODER_DIFF_NO) {
        if (encoder_diffState == ENCODER_DIFF_ENTER) {
          recovery_flag = 0;
          if (HMI_flag.select_flag) break;
          TERN_(POWER_LOSS_RECOVERY, recovery.cancel());
          HMI_StartFrame(true);
          return;
        }
         else if (encoder_diffState == ENCODER_DIFF_CW) {
              Draw_Select_Highlight(false);
             if(HMI_flag.language_flag)
             {
                DWIN_ICON_Show(ICON,43,290,172);
                DWIN_ICON_Show(ICON,45,130,172);
             }
             else
             {
                  DWIN_ICON_Show(ICON,48,130,172);
                  DWIN_ICON_Show(ICON,47,290,172);
             }
        }
        else if (encoder_diffState == ENCODER_DIFF_CCW) { 
           Draw_Select_Highlight(true);
           if(HMI_flag.language_flag)
           {
            DWIN_ICON_Show(ICON,44,130,172);
             DWIN_ICON_Show(ICON,42,290,172);
           }
           else
           {
              DWIN_ICON_Show(ICON,49,130,172);
              DWIN_ICON_Show(ICON,46,290,172);
           }
        }
        DWIN_UpdateLCD();
      }
    }
    select_print.set(0);
    HMI_ValueStruct.show_mode = 0;
    Goto_PrintProcess();
  
    DWIN_UpdateLCD();
    recovery.resume();    
    return;
  }
  DWIN_UpdateLCD();
}

void DWIN_HandleScreen(void) {
  switch (checkkey) {
    case MainMenu:              HMI_MainMenu(); break;
    case SelectFile:            HMI_SelectFile(); break;
    case Prepare:               HMI_Prepare(); break;
    case Control:               HMI_Control(); break;
    case Leveling:              break;
    case PrintProcess:          HMI_Printing(); break;
    case Print_window:          HMI_PauseOrStop(); break;
    case AxisMove:              HMI_AxisMove(); break;
    case TemperatureID:         HMI_Temperature(); break;
    case Motion:                HMI_Motion(); break;
    case Info:                  HMI_Info(); break;
    case Tune:                  HMI_Tune(); break;
    case PLAPreheat:            HMI_PLAPreheatSetting(); break;
    case ABSPreheat:            HMI_ABSPreheatSetting(); break;
    case MaxSpeed:              HMI_MaxSpeed(); break;
    case MaxAcceleration:       HMI_MaxAcceleration(); break;
    case MaxCorner:             HMI_MaxCorner(); break;
    case Step:                  HMI_Step(); break;
    case Move_X:                HMI_Move_X(); break;
    case Move_Y:                HMI_Move_Y(); break;
    case Move_Z:                HMI_Move_Z(); break;
    case Extruder:              HMI_Move_E(); break;
    case Homeoffset:            HMI_Zoffset(); break;
    #if HAS_HOTEND
      case ETemp:               HMI_ETemp(); break;
    #endif
    #if HAS_HEATED_BED
      case BedTemp:             HMI_BedTemp(); break;
    #endif
    #if HAS_FAN
      case FanSpeed:            HMI_FanSpeed(); break;
    #endif
    case PrintSpeed:            HMI_PrintSpeed(); break;
    case MaxSpeed_value:        HMI_MaxFeedspeedXYZE(); break;
    case MaxAcceleration_value: HMI_MaxAccelerationXYZE(); break;
    case MaxCorner_value:       HMI_MaxCornerXYZE(); break;
    case Step_value:            HMI_StepXYZE(); break;
    case Feeding:               HMI_Matic();break;
    case Desizing:              HMI_unload();break;
    case Bltouch:               HMI_Bltouch();break;
    case filament:              HMI_filament();break;
    case filament_exist:        HMI_filament_exist();break;
    default: break;
  }
}

void DWIN_CompletedHoming(void) {
  HMI_flag.home_flag = false;
  if (checkkey == Last_Control) {
    checkkey = Control;
    select_control.reset();
    select_control.now = 3;
    index_control = MROWS;
   // Clear_Menu_Area();
    Draw_Control_Menu();
  }
  else if (checkkey == Back_Main) {
    HMI_ValueStruct.print_speed = feedrate_percentage = 100;
    zprobe_zoffset = TERN0(BLTOUCH, probe.offset.z);
     planner.finish_and_disable();
    Goto_MainMenu();
  }
}

void DWIN_CompletedMatic(void)
{
     if(checkkey==Feeding) Goto_MainMenu();
     if(checkkey==Desizing) Goto_MainMenu();
      //queue.inject_P(PSTR("M104 S0"));      
      //DWIN_Draw_IntValue(true, true, 0, font8x16, Black, Background_black, 3, 50 + 4 * STAT_CHR_W + 6, 200, 0);  
}
void DWIN_CompletedMaticHate(void)
{
      if(checkkey==Feeding) 
      {
        if(HMI_flag.language_flag)
        {
          DWIN_ICON_Show(ICON1,9,140,50);
        }
        else
        {
          DWIN_ICON_Show(ICON1,6,140,50);
        }   
        queue.enqueue_now_P(PSTR("G92 Z0 E0"));
        queue.enqueue_now_P(PSTR("G1 F800 Z10"));
        queue.enqueue_now_P(PSTR("G1 F500 E350"));
        queue.enqueue_now_P(PSTR("G1 F100 E500"));
        queue.enqueue_now_P(PSTR("M400"));
        queue.enqueue_now_P(PSTR("M104 S0"));
        
      }
      if(checkkey==Desizing) 
      {
        if(HMI_flag.language_flag)
        {
         DWIN_ICON_Show(ICON1,2,140,50);
        }
        else
        {
           DWIN_ICON_Show(ICON1,4,140,50);
        }
        queue.enqueue_now_P(PSTR("G92 Z0 E0"));
        queue.enqueue_now_P(PSTR("G1 F800 Z10"));
        queue.enqueue_now_P(PSTR("G1 F500 E50"));
        queue.enqueue_now_P(PSTR("G1 F800 E-600")); 
        queue.enqueue_now_P(PSTR("M400"));
        queue.enqueue_now_P(PSTR("M104 S0"));    
      }
       DWIN_UpdateLCD();
}
void DWIN_CompletedLeveling(void) {
  //if (checkkey == Leveling) Goto_MainMenu();
  if(HMI_flag.language_flag)
  {
      DWIN_ICON_Show(ICON1,32,140,50);
  }
  else
  {
      DWIN_ICON_Show(ICON1,33,140,50);
  }
  completedlevel_flag= true;
  queue.enqueue_now_P(PSTR("M500"));
}


void BLtouch_detecting(int x,int y)
{
      //DWIN_ICON_Show(ICON1,11,240,108);
      if(y==0)
      {
        DWIN_ICON_Show(ICON1,10+4-x,240+28*(3-x),80+26*y);
      }
      else if(y==1)
      {
         DWIN_ICON_Show(ICON1,10+5+x,240+28*(3-x),80+26*y);
      }
      else if(y==2)
      {
         DWIN_ICON_Show(ICON1,18+4-x,240+28*(3-x),80+26*y);
      }
      else if(y==3)
      {
         DWIN_ICON_Show(ICON1,18+5+x,240+28*(3-x),80+26*y);
      }
}
void Filament_Recovery_Flag(void)
{
   filament_recovery_flag=false;
}
#endif // DWIN_CREALITY_LCD