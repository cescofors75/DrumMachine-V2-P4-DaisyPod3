// Fixed per-pad libraries on the P4 SD. Listing is read-only; loading is
// serialized through the existing P4 -> Daisy WAV upload worker and waits for
// the user to stop turning for 350 ms.
static const char* XTRA_DIRECTORIES[4]={
    "/data/xtra/DRUMS",
    "/data/xtra/VOCALS",
    "/data/xtra/FX",
    "/data/xtra/MUSIC"
};
static const char* XTRA_DIRECTORY_LABELS[4]={"DRUMS","VOCALS","FX","MUSIC"};
struct XtraDirectory {
    char files[64][64]{};
    int count=0, selected=-1;
    bool ready=false, pending=false;
    uint32_t changed=0;
};
static XtraDirectory s_xtra_directories[4];
static uint32_t s_xtra_load_sent=0;

static bool xtra_local_scan_request(void);
static void xtra_local_scan_poll(void);
static bool xtra_local_upload_start(int slot,const char* folder,const char* filename);

static void xtra_directory_choose(int slot,int delta) {
    auto& dir=s_xtra_directories[slot];
    if(!dir.ready || !dir.count) return;
    int index=dir.selected;
    if(index<0) index=delta>0 ? -1 : 0;
    dir.selected=((index+delta)%dir.count+dir.count)%dir.count;
    dir.pending=true; dir.changed=millis();
}
static void xtra_directories_start() {
    if(!xtra_local_scan_request()) return;
    s_xtra_page_req=XTRA_PAGE_REQ_NONE;
    for(auto& dir:s_xtra_directories) { dir.ready=false; dir.pending=false; }
    if(s_xtra_page_prev_btn) lv_obj_add_flag(s_xtra_page_prev_btn,LV_OBJ_FLAG_HIDDEN);
    if(s_xtra_page_next_btn) lv_obj_add_flag(s_xtra_page_next_btn,LV_OBJ_FLAG_HIDDEN);
    if(s_xtra_page_lbl) lv_label_set_text(s_xtra_page_lbl,"R1 DRUMS  /  R2 VOCALS  /  R3 FX  /  R4 MUSIC");
}
static void xtra_directories_poll() {
    xtra_local_scan_poll();
    const uint32_t now=millis();
    if(now-s_xtra_load_sent<700) return;
    for(int slot=0;slot<4;++slot) {
        auto& dir=s_xtra_directories[slot];
        if(!dir.pending || dir.selected<0 || now-dir.changed<350 || s_xtra_touch_active[slot]) continue;
        if(!xtra_local_upload_start(slot,XTRA_DIRECTORIES[slot],dir.files[dir.selected])) return;
        dir.pending=false;
        s_xtra_load_sent=now;
        return;
    }
}
