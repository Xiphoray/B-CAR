#ifndef	_LCD_H_
#define _LCD_H_

#define RESET  PTC15_OUT
#define DC     PTC17_OUT
#define D1     PTC13_OUT //÷√∏ﬂ
#define D0     PTC11_OUT
/*#define CS     PTB19_OUT*/
 typedef unsigned char byte;
 typedef unsigned int word;
 extern byte beyond96x64[512];
 void LCD_WrDat(uint8 data);
 void LCD_Set_Pos(byte x, byte y);
 void LCD_Init(void);
 void LCD_CLS(void);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);
 void Display(int16 i,uint8 x,uint8 y);
 void DisplayFloat(int16 i,uint8 x,uint8 y);
 void Oprintfloat(unsigned char x,unsigned char y,float val);
 char* oledftoa(float val,char * string);
#endif