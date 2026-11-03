#include <graphics.h> 
#include <conio.h> 
#include <dos.h> 
#include <math.h> 
#include <stdio.h>
#define SENSORS 8
#define BASE 190 /* cruising PWM (0-255) */ 
#define SPACING 8 /* pixels between two sensors */ 
#define AHEAD 18 /* sensor bar distance in front of body */ 
#define AMP 60 /* line amplitude in pixels */ 
#define WAVE 100.0 /* line wavelength factor */
/* black line path drawn on the screen */ 
float lineY(float x)
{
return 240 + AMP * sin(x / WAVE);
}
/* IR sensor model: high reading over the line, low (white) away from it */ 
int sensorValue(float px, float py)
{
float slope = AMP / WAVE * cos(px / WAVE);
float d = fabs(py - lineY(px)) / sqrt(1 + slope * slope); /* true distance */ 
if (d > 10) return 50;
return (int)(50 + 850 * (1 - d / 10));
}
/* a = distance along robot heading, b = distance to its right */
void toWorld(float x, float y, float th, float a, float b, float *px, float *py)
{
*px = x + a * cos(th) - b * sin(th);
*py = y + a * sin(th) + b * cos(th);
}
void drawRobot(float x, float y, float th)
{
float ax, ay, bx, by; 
int i;
toWorld(x, y, th, -12, -18, &ax, &ay); /* body rectangle */ 
toWorld(x, y, th, -12, 18, &bx, &by);
line((int)ax, (int)ay, (int)bx, (int)by);
toWorld(x, y, th, 12, 18, &ax, &ay); 
line((int)bx, (int)by, (int)ax, (int)ay);
toWorld(x, y, th, 12, -18, &bx, &by);
line((int)ax, (int)ay, (int)bx, (int)by);
toWorld(x, y, th, -12, -18, &ax, &ay);
line((int)bx, (int)by, (int)ax, (int)ay);
for (i = 0; i < SENSORS; i++) /* 8 sensors */
{
toWorld(x, y, th, AHEAD, (i - 3.5) * SPACING, &ax, &ay); 
rectangle((int)ax - 1, (int)ay - 1, (int)ax + 1, (int)ay + 1);
}
}
void main()
{
int gd = DETECT, gm, i, px, py, sv, black, line_pos; 
int x, left, right, quit = 0;
long suma, sgain;
float rx = 20, ry, th, sx, sy, v; 
char msg[60];
initgraph(&gd, &gm, "C:\\TURBOC3\\BGI"); /* change to your BGI folder */ 
if (graphresult() != grOk)
{
printf("Graphics error: check the BGI path"); 
getch();
return;
}
/* draw the track */ 
setcolor(WHITE);
for (px = 0; px < 640; px++)
{
py = (int)lineY(px); 
putpixel(px, py - 1, WHITE); 
putpixel(px, py, WHITE); 
putpixel(px, py + 1, WHITE);
}
outtextxy(10, 10, "Line Following Robot Simulation (press any key to quit)");
ry = lineY(rx);
th = atan(AMP / WAVE * cos(rx / WAVE)); /* start along the line */ 
setcolor(LIGHTGREEN);
setwritemode(XOR_PUT); /* draw twice = erase */ 
drawRobot(rx, ry, th);
while (!kbhit() && !quit)
{
drawRobot(rx, ry, th); /* erase old robot */
suma = 0; sgain = 0; black = 0; /* read 8 sensors */ 
for (i = 0; i < SENSORS; i++)
{
toWorld(rx, ry, th, AHEAD, (i - 3.5) * SPACING, &sx, &sy); 
sv = sensorValue(sx, sy);
suma += sv;
sgain += (long)sv * (i + 1) * 10; 
if (sv > 300) black++;
}
setwritemode(COPY_PUT); 
setfillstyle(SOLID_FILL, BLACK); 
bar(0, 440, 639, 479);
setcolor(WHITE);
if (black == 0) /* line lost */
{
outtextxy(10, 445, "Line lost - STOP Robot"); 
quit = 1;
}
else
{
line_pos = (int)(sgain / suma); /* 10 .. 80 */ 
x = 45 - line_pos; /* -35 .. +35 */ 
left = BASE - 2 * x;
right = BASE + 2 * x;
if (left > 255) left = 255; 
if (left < 0) left = 0;
if (right > 255) right = 255; 
if (right < 0) right = 0;
sprintf(msg, "x=%3d Left PWM=%3d Right PWM=%3d", x, left, right); 
outtextxy(10, 445, msg);
if (x >= -5 && x <= 5) outtextxy(10, 460, "Action: Move FORWARD"); 
else if (x > 5) outtextxy(10, 460, "Action: Turn LEFT");
else outtextxy(10, 460, "Action: Turn RIGHT");
th += (left - right) * 0.0012; /* differential drive */ 
v = (left + right) / 2.0 * 0.025;
rx += v * cos(th); 
ry += v * sin(th);
putpixel((int)rx, (int)ry, YELLOW); /* path trail */ 
if (rx > 620) quit = 1;
}
setcolor(LIGHTGREEN); 
setwritemode(XOR_PUT);
drawRobot(rx, ry, th); /* draw new robot */ 
delay(50);
}
setwritemode(COPY_PUT); 
setcolor(WHITE);
if (kbhit()) getch();
outtextxy(300, 460, "STOP - press any key to exit"); 
getch();
closegraph();
}