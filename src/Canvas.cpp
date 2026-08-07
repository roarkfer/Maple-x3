#include "Canvas.h"
#include "Font5x7.h"
#include <cstdlib>
void Canvas::pixel(int x,int y,bool black){if(x<0||y<0||x>=w||y>=h)return;int px=x,py=y;if(orientation==Rotation::Clockwise){px=physicalW-1-y;py=x;}else if(orientation==Rotation::CounterClockwise){px=y;py=physicalH-1-x;}if(px<0||py<0||px>=physicalW||py>=physicalH)return;uint8_t& cell=data[py*stride+px/8];uint8_t mask=0x80>>(px%8);if(black)cell&=~mask;else cell|=mask;}
void Canvas::line(int x0,int y0,int x1,int y1){int dx=abs(x1-x0),sx=x0<x1?1:-1;int dy=-abs(y1-y0),sy=y0<y1?1:-1;int e=dx+dy;while(true){pixel(x0,y0);if(x0==x1&&y0==y1)break;int e2=2*e;if(e2>=dy){e+=dy;x0+=sx;}if(e2<=dx){e+=dx;y0+=sy;}}}
void Canvas::rect(int x,int y,int ww,int hh,bool fill){if(fill){for(int yy=y;yy<y+hh;++yy)for(int xx=x;xx<x+ww;++xx)pixel(xx,yy);return;}line(x,y,x+ww-1,y);line(x,y+hh-1,x+ww-1,y+hh-1);line(x,y,x,y+hh-1);line(x+ww-1,y,x+ww-1,y+hh-1);}
void Canvas::character(int x,int y,char c,uint8_t s){if(c<32||c>126)c='?';const uint8_t* g=FONT_5X7[c-32];for(int col=0;col<5;++col)for(int row=0;row<7;++row)if(pgm_read_byte(&g[col])&(1<<row))rect(x+col*s,y+row*s,s,s,true);}
void Canvas::text(int x,int y,const char* v,uint8_t s){while(*v){character(x,y,*v++,s);x+=6*s;}}
int Canvas::textWidth(const char* v,uint8_t s)const{return strlen(v)*6*s;}
