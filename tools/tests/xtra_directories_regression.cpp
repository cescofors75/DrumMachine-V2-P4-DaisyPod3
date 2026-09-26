#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include "../../P4/src/master/protocol.h"
static uint32_t now=1000;
uint32_t millis(){return now;}
using lv_obj_t=int;
static lv_obj_t *s_xtra_page_prev_btn=nullptr,*s_xtra_page_next_btn=nullptr,*s_xtra_page_lbl=nullptr;
constexpr int LV_OBJ_FLAG_HIDDEN=1,RED808_WARNING=0,XTRA_PAGE_REQ_NONE=0;
static int s_xtra_page_req=0;
void lv_obj_add_flag(lv_obj_t*,int){}
void lv_label_set_text(lv_obj_t*,const char*){}
void ui_show_toast(const char*,int){}
bool ui_control_available(){return true;}
struct Slot { bool used=false,synth_mode=false; uint8_t pad=0; char name[32]{};
int trim_start_pct=0,trim_end_pct=100,duration_ms=0,sample_rate=0,channels=0,bits=0; };
static Slot s_xtra_slots[4];
static bool s_xtra_touch_active[4]{};
static int s_xtra_wave_count[4]{};
uint8_t xtra_backing_pad_for_slot(int i){return 16+i;}
void trim_wav_extension(char* p){ auto n=strlen(p); if(n>4)p[n-4]=0; }
void xtra_save_state(){}
void xtra_refresh_panel(){}
struct Transport {
    struct State { uint32_t daisy_sd_files_revision=0; uint16_t daisy_sd_files_sequence=0;
        int daisy_sd_file_count=0; char daisy_sd_files[20][32]{}; } st;
    int requests=0,loads=0; SdLoadSamplePayload last{}; char folder[96]{};
    const State& state(){return st;}
    bool requestFileList(const char* f,uint16_t& seq){strcpy(folder,f);seq=++requests;return true;}
    bool send(uint8_t cmd,const void* p,size_t){assert(cmd==CMD_SD_LOAD_SAMPLE);memcpy(&last,p,sizeof(last));++loads;return true;}
} daisyUsb;
static int scanRequests=0,uploads=0,lastSlot=-1;
static char lastFolder[96]{},lastFile[64]{};
static bool xtra_local_scan_request(){++scanRequests;return true;}
static void xtra_local_scan_poll(){}
static bool xtra_local_upload_start(int slot,const char* folder,const char* filename){
    ++uploads;lastSlot=slot;strcpy(lastFolder,folder);strcpy(lastFile,filename);return true;
}
#include "../../P4/src/ui/ui_xtra_directories.inc"
int main(){
    strcpy(s_xtra_slots[1].name,"keep");
    xtra_directories_start();
    assert(scanRequests==1 && !strcmp(XTRA_DIRECTORIES[0],"/data/xtra/DRUMS"));
    auto& drums=s_xtra_directories[0];
    drums.ready=true;drums.count=2;
    strcpy(drums.files[0],"a.wav");
    strcpy(drums.files[1],"WavePoint-CowbellLoop1-123bpm.wav");
    xtra_directory_choose(0,1); xtra_directory_choose(0,1);
    xtra_directories_poll(); assert(uploads==0);
    now+=350; s_xtra_touch_active[0]=true;
    xtra_directories_poll(); assert(uploads==0);
    s_xtra_touch_active[0]=false; xtra_directories_poll();
    assert(uploads==1 && lastSlot==0);
    assert(!strcmp(lastFolder,"/data/xtra/DRUMS"));
    assert(!strcmp(lastFile,"WavePoint-CowbellLoop1-123bpm.wav"));
    assert(!drums.pending && !strcmp(s_xtra_slots[1].name,"keep"));
    puts("XTRA: P4 SD paths, long WAV names, wrap, debounce and held pads PASS");
}
