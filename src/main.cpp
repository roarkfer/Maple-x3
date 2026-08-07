#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <BoardConfig.h>
#include <EInkDisplay.h>
#include <InputManager.h>
#include <PowerManager.h>
#include <XteinkDetect.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include "Canvas.h"

namespace {
constexpr gpio_num_t X3_POWER_LATCH=GPIO_NUM_13;
constexpr uint8_t FULL_REFRESH_EVERY=10;
constexpr int MAX_TASKS=24,MAX_HABITS=16,MAX_FOLDERS=10,MAX_EXERCISES=16;
struct Task{String text;bool done=false;};
struct Habit{String name;uint32_t marks=0;}; // rolling 31-day bitmap
struct Exercise{String name;int sets=4,reps=12,kg=10;};
struct Folder{String name;Exercise ex[MAX_EXERCISES];int count=0;};
Task tasks[MAX_TASKS]; Habit habits[MAX_HABITS]; Folder folders[MAX_FOLDERS];
int taskCount=0,habitCount=0,folderCount=0;

enum class Tab:uint8_t{Habits,Tasks,Exercises};
enum class Screen:uint8_t{List,Folder,Keyboard};
Tab tab=Tab::Tasks; Screen screen=Screen::List;
int cursor=0,openFolder=-1; bool editing=false; uint8_t fastRefreshes=0;
String* editTarget=nullptr; String keyboardBuffer; int keyCursor=0;
const char KEYS[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -_.";
Preferences prefs;

EInkDisplay display(BoardConfig::ACTIVE.display.sclk,BoardConfig::ACTIVE.display.mosi,
 BoardConfig::ACTIVE.display.cs,BoardConfig::ACTIVE.display.dc,
 BoardConfig::ACTIVE.display.rst,BoardConfig::ACTIVE.display.busy);
InputManager buttons;

void beginDisplayHardware(){SPI.end();SPI.begin(BoardConfig::ACTIVE.display.sclk,BoardConfig::ACTIVE.sd.miso,BoardConfig::ACTIVE.display.mosi,BoardConfig::ACTIVE.display.cs);display.begin();}
void centered(Canvas& c,int y,const char* t,uint8_t s){c.text((c.width()-c.textWidth(t,s))/2,y,t,s);}
void saveData(){
  JsonDocument d;
  auto ta=d["tasks"].to<JsonArray>(); for(int i=0;i<taskCount;i++){auto o=ta.add<JsonObject>();o["t"]=tasks[i].text;o["d"]=tasks[i].done;}
  auto ha=d["habits"].to<JsonArray>(); for(int i=0;i<habitCount;i++){auto o=ha.add<JsonObject>();o["n"]=habits[i].name;o["m"]=habits[i].marks;}
  auto fa=d["folders"].to<JsonArray>(); for(int i=0;i<folderCount;i++){auto fo=fa.add<JsonObject>();fo["n"]=folders[i].name;auto ea=fo["e"].to<JsonArray>();for(int j=0;j<folders[i].count;j++){auto e=ea.add<JsonObject>();e["n"]=folders[i].ex[j].name;e["s"]=folders[i].ex[j].sets;e["r"]=folders[i].ex[j].reps;e["k"]=folders[i].ex[j].kg;}}
  String json;serializeJson(d,json);prefs.putString("data",json);
}
void loadData(){
 prefs.begin("maple",false);String json=prefs.getString("data","");
 if(!json.length())return;JsonDocument d;if(deserializeJson(d,json))return;
 taskCount = 0;
for (JsonObject o : d["tasks"].as<JsonArray>()) {
  if (taskCount >= MAX_TASKS) break;

  tasks[taskCount].text = String(o["t"] | "");
  tasks[taskCount].done = o["d"] | false;
  taskCount++;
}

habitCount = 0;
for (JsonObject o : d["habits"].as<JsonArray>()) {
  if (habitCount >= MAX_HABITS) break;

  habits[habitCount].name = String(o["n"] | "");
  habits[habitCount].marks = o["m"] | 0u;
  habitCount++;
}

folderCount = 0;
for (JsonObject fo : d["folders"].as<JsonArray>()) {
  if (folderCount >= MAX_FOLDERS) break;

  Folder& f = folders[folderCount++];
  f.name = String(fo["n"] | "");
  f.count = 0;

  for (JsonObject e : fo["e"].as<JsonArray>()) {
    if (f.count >= MAX_EXERCISES) break;

    f.ex[f.count].name = String(e["n"] | "");
    f.ex[f.count].sets = e["s"] | 4;
    f.ex[f.count].reps = e["r"] | 12;
    f.ex[f.count].kg = e["k"] | 10;
    f.count++;
  }
}
}
template<typename T> void moveItem(T* a,int n,int from,int to){if(to<0||to>=n)return;T tmp=a[from];if(to<from)for(int i=from;i>to;--i)a[i]=a[i-1];else for(int i=from;i<to;++i)a[i]=a[i+1];a[to]=tmp;}
int itemCount(){if(tab==Tab::Tasks)return taskCount;if(tab==Tab::Habits)return habitCount;if(openFolder>=0)return folders[openFolder].count;return folderCount;}
const char* tabName(){return tab==Tab::Habits?"HABITOS":tab==Tab::Tasks?"TAREAS":"EJERCICIOS";}
void drawTabs(Canvas& c){
 const char* names[3]={"HABITOS","TAREAS","EJERCICIOS"};for(int i=0;i<3;i++){int x=18+i*170;bool on=(int)tab==i;c.rect(x,18,156,54,on);if(on){/* inverted text unavailable; use outline marker */c.rect(x+4,22,148,46,false);}c.text(x+16,37,names[i],1);}c.line(18,84,510,84);
}
void drawList(Canvas& c){
 drawTabs(c);c.text(24,105,tabName(),2);c.text(365,108,editing?"EDITAR":"POWER: EDIT",1);
 int n=itemCount(); if(!n){centered(c,330,"LISTA VACIA",2);centered(c,365,"POWER PARA EDITAR",1);return;}
 int start=max(0,cursor-5),end=min(n,start+7);if(end-start<7)start=max(0,end-7);
 for(int row=0,i=start;i<end;i++,row++){int y=155+row*72;bool sel=i==cursor;if(sel)c.rect(18,y-12,492,58,false);char line[64]={};
  if(tab==Tab::Tasks){snprintf(line,sizeof(line),"%s %s",tasks[i].done?"[X]":"[ ]",tasks[i].text.c_str());}
  else if(tab==Tab::Habits){snprintf(line,sizeof(line),"%s  %s", (habits[i].marks&1)?"[X]":"[ ]", habits[i].name.c_str());}
  else if(openFolder<0){snprintf(line,sizeof(line),"[%02d] %s",folders[i].count,folders[i].name.c_str());}
  else {Exercise&e=folders[openFolder].ex[i];snprintf(line,sizeof(line),"%s  S:%d R:%d KG:%d",e.name.c_str(),e.sets,e.reps,e.kg);}
  c.text(30,y,line,2);
 }
 c.line(18,700,510,700);c.text(24,720,editing?"< > REORDENA   OK EDITA   BACK LISTO":"< > PESTANA   OK ACCION   POWER EDITA",1);
}
void drawKeyboard(Canvas& c){
 c.text(24,26,"EDITAR TEXTO",2);c.rect(20,70,488,58);String shown=keyboardBuffer;if(shown.length()>32)shown=shown.substring(shown.length()-32);c.text(30,90,shown.c_str(),2);
 const int cols=7;int count=strlen(KEYS);for(int i=0;i<count;i++){int r=i/cols,cc=i%cols;int x=28+cc*70,y=180+r*62;if(i==keyCursor)c.rect(x-8,y-14,54,48);char s[2]={KEYS[i],0};c.text(x,y,s,2);}
 int y=180+((count+cols-1)/cols)*62+10;c.text(30,y,"OK: CARACTER   POWER: GUARDAR",1);c.text(30,y+28,"BACK: BORRAR   HOLD POWER: DORMIR",1);
}
void render(bool full=false){display.clearScreen();Canvas c(display.getFrameBuffer(),display.getDisplayWidth(),display.getDisplayHeight(),Canvas::Rotation::CounterClockwise);if(screen==Screen::Keyboard)drawKeyboard(c);else drawList(c);bool f=full||fastRefreshes>=FULL_REFRESH_EVERY;display.displayBuffer(f?EInkDisplay::FULL_REFRESH:EInkDisplay::FAST_REFRESH,true);fastRefreshes=f?0:fastRefreshes+1;}
void startEdit(String& s){editTarget=&s;keyboardBuffer=s;keyCursor=0;screen=Screen::Keyboard;render();}
void addNew(){
 if(tab==Tab::Tasks&&taskCount<MAX_TASKS){tasks[taskCount++].text="NUEVA TAREA";cursor=taskCount-1;startEdit(tasks[cursor].text);}
 else if(tab==Tab::Habits&&habitCount<MAX_HABITS){habits[habitCount++].name="NUEVO HABITO";cursor=habitCount-1;startEdit(habits[cursor].name);}
 else if(tab==Tab::Exercises&&openFolder<0&&folderCount<MAX_FOLDERS){folders[folderCount].name="NUEVA CARPETA";folders[folderCount].count=0;cursor=folderCount++;startEdit(folders[cursor].name);}
 else if(tab==Tab::Exercises&&openFolder>=0&&folders[openFolder].count<MAX_EXERCISES){Folder&f=folders[openFolder];f.ex[f.count].name="NUEVO EJERCICIO";cursor=f.count++;startEdit(f.ex[cursor].name);}
 saveData();
}
void shortPower(){editing=!editing;render();}
void enterSleep(){saveData();display.deepSleep();freeink::PowerManager::waitForPowerButtonRelease();freeink::PowerManager::armPowerButtonWakeup();freeink::PowerManager::powerDownRailsForSleep();gpio_set_direction(X3_POWER_LATCH,GPIO_MODE_OUTPUT);gpio_set_level(X3_POWER_LATCH,0);gpio_hold_en(X3_POWER_LATCH);freeink::PowerManager::deepSleep();}
void handleKeyboard(){
 int cols=7,count=strlen(KEYS);
 if(buttons.wasPressed(InputManager::BTN_LEFT)){keyCursor=(keyCursor+count-1)%count;render();}
 if(buttons.wasPressed(InputManager::BTN_RIGHT)){keyCursor=(keyCursor+1)%count;render();}
 if(buttons.wasPressed(InputManager::BTN_UP)){keyCursor=(keyCursor-cols+count)%count;render();}
 if(buttons.wasPressed(InputManager::BTN_DOWN)){keyCursor=(keyCursor+cols)%count;render();}
 if(buttons.wasPressed(InputManager::BTN_CONFIRM)){if(keyboardBuffer.length()<36)keyboardBuffer+=KEYS[keyCursor];render();}
 if(buttons.wasPressed(InputManager::BTN_BACK)){if(keyboardBuffer.length())keyboardBuffer.remove(keyboardBuffer.length()-1);render();}
 if(buttons.wasPressed(InputManager::BTN_POWER)){if(editTarget){*editTarget=keyboardBuffer;saveData();}editTarget=nullptr;screen=Screen::List;editing=true;render();}
}
void handleList(){
 int n=itemCount();
 if(buttons.wasPressed(InputManager::BTN_UP)){if(n){cursor=(cursor+n-1)%n;render();}}
 if(buttons.wasPressed(InputManager::BTN_DOWN)){if(n){cursor=(cursor+1)%n;render();}}
 if(buttons.wasPressed(InputManager::BTN_LEFT)){
   if(editing&&n){if(cursor>0){if(tab==Tab::Tasks)moveItem(tasks,taskCount,cursor,cursor-1);else if(tab==Tab::Habits)moveItem(habits,habitCount,cursor,cursor-1);else if(openFolder<0)moveItem(folders,folderCount,cursor,cursor-1);else moveItem(folders[openFolder].ex,folders[openFolder].count,cursor,cursor-1);cursor--;saveData();render();}}
   else {tab=Tab((int(tab)+2)%3);openFolder=-1;cursor=0;render();}
 }
 if(buttons.wasPressed(InputManager::BTN_RIGHT)){
   if(editing&&n){if(cursor<n-1){if(tab==Tab::Tasks)moveItem(tasks,taskCount,cursor,cursor+1);else if(tab==Tab::Habits)moveItem(habits,habitCount,cursor,cursor+1);else if(openFolder<0)moveItem(folders,folderCount,cursor,cursor+1);else moveItem(folders[openFolder].ex,folders[openFolder].count,cursor,cursor+1);cursor++;saveData();render();}}
   else {tab=Tab((int(tab)+1)%3);openFolder=-1;cursor=0;render();}
 }
 if(buttons.wasPressed(InputManager::BTN_CONFIRM)){
   if(editing){if(!n)addNew();else if(tab==Tab::Tasks)startEdit(tasks[cursor].text);else if(tab==Tab::Habits)startEdit(habits[cursor].name);else if(openFolder<0)startEdit(folders[cursor].name);else startEdit(folders[openFolder].ex[cursor].name);}
   else if(n){if(tab==Tab::Tasks){tasks[cursor].done=!tasks[cursor].done;saveData();render();}else if(tab==Tab::Habits){habits[cursor].marks^=1;saveData();render();}else if(openFolder<0){openFolder=cursor;cursor=0;render();}else {Exercise&e=folders[openFolder].ex[cursor];e.reps++;saveData();render();}}
 }
 if(buttons.wasPressed(InputManager::BTN_BACK)){
   if(editing){editing=false;render();}else if(openFolder>=0){openFolder=-1;cursor=0;render();}
 }
 if(buttons.wasPressed(InputManager::BTN_POWER))shortPower();
}
}
void setup(){
 delay(250);gpio_deep_sleep_hold_dis();gpio_hold_dis(X3_POWER_LATCH);pinMode(X3_POWER_LATCH,OUTPUT);digitalWrite(X3_POWER_LATCH,HIGH);Serial.begin(115200);
 auto verdict=freeink::detectX3DisplayController();bool uc=verdict==freeink::X3DisplayVerdict::Uc8279Confirmed;BoardConfig::selectDevice(uc?BoardConfig::Board::XteinkX3Uc8279:BoardConfig::Board::XteinkX3);display.setDisplayX3();BoardConfig::releaseSdRail();buttons.begin();for(int i=0;i<4;i++){buttons.update();delay(25);}loadData();beginDisplayHardware();display.requestResync();render(true);freeink::PowerManager::waitForPowerButtonRelease();buttons.update();
}
void loop(){
 buttons.update();if(buttons.isPressed(InputManager::BTN_POWER)&&buttons.getPowerButtonHeldTime()>1200){enterSleep();}
 if(screen==Screen::Keyboard)handleKeyboard();else handleList();delay(10);
}
