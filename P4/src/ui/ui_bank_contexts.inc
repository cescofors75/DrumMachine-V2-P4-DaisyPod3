// Included by ui_screens.cpp after its screen adapters. All LVGL access stays
// on that task; the hardware mailbox contains no widget pointers or callbacks.
namespace {
using BP = bank4::Param;
using BT = bank4::Type;
enum BankId : uint16_t {
    B_LEVEL=1, B_PAN, B_REVERB, B_DELAY, B_MUTE, B_SOLO, B_SOURCE,
    B_PITCH, B_VELOCITY, B_PROB, B_REPEAT, B_GATE, B_ACCENT, B_MOD,
    B_CUTOFF, B_RESO, B_DRIVE, B_BITS, B_FADE_IN, B_FADE_OUT,
    B_OCTAVE, B_PIANO_VEL, B_PIANO_ENGINE, B_PIANO_VOL,
    B_FX=100, B_SYNTH=200
};
static bank4::Controller s_bank;
static bank4::Bank s_banks[10];
static int s_bank_pad=0, s_bank_engine=0;
static lv_obj_t* s_bank_dock=nullptr;
static lv_obj_t* s_bank_title=nullptr;
static lv_obj_t* s_bank_names[4]={};
static lv_obj_t* s_bank_values[4]={};
static lv_obj_t* s_bank_cells[4]={};
static lv_obj_t* s_bank_pad_buttons[2]={};
static uint32_t s_bank_highlight[4]={};
static char s_bank_context_name[64];
static bool s_bank_defaults_ready=false;

static float bank_get(void*, uint16_t id) {
    if(id>=B_SYNTH) return pp_param_value_or_default(s_bank_engine,id-B_SYNTH,0);
    if(id>=B_FX) {
        const int cell=id-B_FX;
        if(cell==8) return p4.distortion_pct;
        if(cell==9) return p4.bitcrush_bits;
        if(cell==11) return p4.filter_type;
        return fx_card_current_value_u7(cell);
    }
    const auto& pad=bank_state::pads[s_bank_pad];
    const auto& fx=pad_fx_state_for(s_bank_pad);
    switch(id) {
        case B_LEVEL:return p4.track_volume[s_bank_pad];
        case B_PAN:return pad.pan.load();
        case B_REVERB:return pad_fx_percent(fx.rvbU7);
        case B_DELAY:return pad_fx_percent(fx.dlyU7);
        case B_MUTE:return p4.track_muted[s_bank_pad];
        case B_SOLO:return p4.track_solo[s_bank_pad];
        case B_SOURCE:return s_pad_inst_sel[s_bank_pad];
        case B_PITCH:return pad.pitch.load();
        case B_VELOCITY:return pad.velocityEnabled.load() ? pad.velocity.load() : ui_live_pad_velocity();
        case B_PROB:return pad.probability.load();
        case B_REPEAT:return pad.repeat.load();
        case B_GATE:return pad.gate.load();
        case B_ACCENT:return pad.accent.load();
        case B_MOD:return pad.mod.load();
        case B_CUTOFF:return fx.cutoffU7;
        case B_RESO:return fx.resoU7;
        case B_DRIVE:return fx.driveU7;
        case B_BITS:return pad_fx_bits(fx.bitsU7);
        case B_FADE_IN:return pad.fadeIn.load();
        case B_FADE_OUT:return pad.fadeOut.load();
        case B_OCTAVE:return s_piano_octave;
        case B_PIANO_VEL:return s_piano_velocity;
        case B_PIANO_ENGINE:return s_piano_engine_idx;
        case B_PIANO_VOL:return p4.live_volume;
        default:return 0;
    }
}
static void bank_set(void*, uint16_t id,float value) {
    if(id>=B_SYNTH) {
        const auto& engine=SP_ENGINES[s_bank_engine];
        for(uint8_t i=0;i<engine.param_count;++i) if(engine.params[i].param_id==id-B_SYNTH) {
            s_pp_values[s_bank_engine][i]=value;
            control_send_synth_param(engine.engine,0,engine.params[i].param_id,value);
            if(s_pp_engine_idx==s_bank_engine && scr_piano_params && lv_obj_is_valid(scr_piano_params)) {
                pp_cell_redraw_value(i); pp_refresh_wave_preview();
                if(s_pp_from_xtra && s_pp_xtra_slot>=0) xtra_capture_editor_state(s_pp_xtra_slot);
            }
            return;
        }
        return;
    }
    if(id>=B_FX) {
        const int cell=id-B_FX;
        if(cell<3) p4.enc_muted[cell]=false;
        else if(cell==FX_CARD_FOLD) p4.pot_muted[0]=false;
        else if(cell==FX_CARD_PHASER) p4.pot_muted[2]=false;
        if(cell==8) control_send_set_distortion(value/100.f);
        else if(cell==9) { p4.bitcrush_bits=int(value); control_send_set_bitcrush(int(value)); }
        else if(cell==11) { p4.filter_type=int(value); control_send_set_filter(int(value)); }
        else fx_card_send_value(cell,int(value));
        if(!fx_card_is_muted(cell)) s_fx_last_active_u7[cell]=fx_card_current_value_u7(cell);
        s_fx_arc_anim[cell]=fx_card_is_muted(cell) ? 0.f : float(fx_card_current_value_u7(cell));
        s_fx_arc_user_ms[cell]=millis()-800;
        update_fx_screen(); return;
    }
    const int v=int(lroundf(value));
    auto& pad=bank_state::pads[s_bank_pad];
    auto& fx=pad_fx_state_for(s_bank_pad);
    switch(id) {
        case B_LEVEL:control_send_set_track_volume(s_bank_pad,v); break;
        case B_PAN: {
            pad.pan=v; const uint8_t data[2]={uint8_t(s_bank_pad),uint8_t(int8_t(v))};
            daisyUsb.send(CMD_TRACK_PAN,data,2); break;
        }
        case B_REVERB:fx.rvbU7=int(lroundf(v*127.f/100.f)); control_send_track_reverb_send(s_bank_pad,v); break;
        case B_DELAY:fx.dlyU7=int(lroundf(v*127.f/100.f)); control_send_track_delay_send(s_bank_pad,v); break;
        case B_MUTE:control_send_mute(s_bank_pad,v!=0); break;
        case B_SOLO:control_send_solo(s_bank_pad,v!=0); break;
        case B_PITCH: {
            pad.pitch=v; const int16_t cents=int16_t(v*100);
            const uint8_t data[3]={uint8_t(s_bank_pad),uint8_t(cents),uint8_t(uint16_t(cents)>>8)};
            daisyUsb.send(CMD_PAD_PITCH,data,3); break;
        }
        case B_VELOCITY:pad.velocity=v; pad.velocityEnabled=true; break;
        case B_PROB:pad.probability=v; break;
        case B_REPEAT:pad.repeat=v; break;
        case B_GATE:pad.gate=v; break;
        case B_ACCENT:pad.accent=v; break;
        case B_MOD: {
            pad.mod=v; const uint8_t data[2]={uint8_t(s_bank_pad),uint8_t(v)};
            daisyUsb.send(CMD_TRACK_CHORUS_SEND,data,2); break;
        }
        case B_CUTOFF:fx.cutoffU7=v; if(!fx.filterType) fx.filterType=1; pad_fx_send_filter(s_bank_pad); break;
        case B_RESO:fx.resoU7=v; if(!fx.filterType) fx.filterType=1; pad_fx_send_filter(s_bank_pad); break;
        case B_DRIVE:fx.driveU7=v; control_send_track_distortion(s_bank_pad,v/127.f); break;
        case B_BITS:fx.bitsU7=int(lroundf((16-v)*127.f/12.f)); control_send_track_bitcrush(s_bank_pad,v); break;
        case B_FADE_IN:pad.fadeIn=v; s_pad_fade_in_ms[s_bank_pad]=v; control_send_set_pad_fade_in(s_bank_pad,v); break;
        case B_FADE_OUT:pad.fadeOut=v; s_pad_fade_out_ms[s_bank_pad]=v; control_send_set_pad_fade_out(s_bank_pad,v); break;
        case B_OCTAVE:
            piano_send_off(); s_piano_octave=v; piano_rebuild_keys();
            if(s_piano_octave_lbl) lv_label_set_text_fmt(s_piano_octave_lbl,"OCT %d",v);
            control_send_melody_set_octave(v); piano_publish_local_state(); break;
        case B_PIANO_VEL:s_piano_velocity=v; break;
        case B_PIANO_ENGINE:
            piano_send_off(); piano_send_panic_melodic(); s_piano_engine_idx=v;
            piano_refresh_engine_chips(); control_send_melody_set_engine(PIANO_ENGINES[v]);
            piano_sync_active_engine_state(); piano_publish_local_state(); break;
        case B_PIANO_VOL:control_send_set_live_volume(v); break;
        default:break;
    }
    if(s_pad_fx_modal) pad_fx_modal_refresh();
    if(lv_scr_act()==scr_volumes) update_volumes_screen();
}
static BP bp(uint16_t id,const char* label,float lo,float hi,BT type=BT::Integer,float step=1) {
    return BP{id,label,type,lo,hi,step,lo,theme_presets[ui_theme_index()].accent,0};
}
static void bank_press(uint8_t slot) {
    const auto* p=s_bank.slot(slot);
    if(!p || p->id<B_FX || p->id>=B_SYNTH) { s_bank.press(slot); return; }
    const int cell=p->id-B_FX;
    if(!fx_card_is_muted(cell)) {
        s_fx_last_active_u7[cell]=fx_card_current_value_u7(cell);
        fx_card_turn_off(cell);
    } else {
        if(cell<3) p4.enc_muted[cell]=false;
        else if(cell==FX_CARD_FOLD) p4.pot_muted[0]=false;
        else if(cell==FX_CARD_PHASER) p4.pot_muted[2]=false;
        int value=s_fx_last_active_u7[cell];
        if(value==fx_card_neutral_u7(cell)) value=cell==FX_CARD_CUTOFF ? 96 : 64;
        fx_card_send_value(cell,value);
    }
    s_fx_arc_anim[cell]=fx_card_is_muted(cell) ? 0.f : float(fx_card_current_value_u7(cell));
    s_fx_arc_user_ms[cell]=millis()-800;
    s_bank_highlight[slot]=millis()+500;
    control_mark_fx_screen_dirty(); update_fx_screen();
}
static bool bank_rotate(uint8_t slot,int delta) {
    if(s_bank.rotate(slot,delta)) return true;
    const auto* p=s_bank.slot(slot);
    if(!delta || !p || p->id<B_FX || p->id>=B_SYNTH) return false;
    const int cell=p->id-B_FX;
    // A bypassed effect still activates when its stored value is at the limit.
    if((cell<3 || cell==FX_CARD_FOLD || cell==FX_CARD_PHASER) && fx_card_is_muted(cell)) {
        bank_set(nullptr,p->id,s_bank.value(slot)); return true;
    }
    return false;
}
static BP bank_synth_param(const SynthEngineDef& engine,int id) {
    const auto* p=synthBankParam(engine,id); if(!p) return {};
    const BT type=!strcmp(p->unit,"Hz") ? BT::Log :
        (!strcmp(p->unit,"s") || !strcmp(p->unit,"ms")) ? BT::Time : p->step_int ? BT::Enum : BT::Continuous;
    BP out=bp(B_SYNTH+id,p->name,p->vmin,p->vmax,type,(p->vmax-p->vmin)/100.f);
    out.initial=p->vdef; return out;
}
static void bank_append_synth(uint8_t& count,int engineIndex) {
    if(engineIndex<0 || engineIndex>=SP_ENGINE_COUNT) return;
    const auto& engine=SP_ENGINES[engineIndex]; uint8_t n=0;
    const auto* defs=synthBanks(engine.engine,n);
    for(uint8_t b=0;b<n && count<10;++b) {
        s_banks[count].title=defs[b].title;
        for(uint8_t i=0;i<4;++i) s_banks[count].slots[i]=bank_synth_param(engine,defs[b].ids[i]);
        ++count;
    }
}
static void bank_input_touch(lv_event_t* e) {
    const int action=int(intptr_t(lv_event_get_user_data(e)));
    ui_note_control_activity();
    if(action==-1 || action==-2) {
        s_bank.select(int(s_bank.active)+(action==-1 ? -1 : 1)); bank_input::clear(); return;
    }
    if(action==-3 || action==-4) {
        const uint8_t pad=uint8_t((s_bank_pad+(action==-3 ? 15 : 1))%16);
        s_pad_inst_focus_pad=pad;
        if(s_pad_fx_modal) { s_pad_fx_focus_pad=pad; pad_fx_modal_refresh(); }
        pad_inst_refresh_controls(); if(s_pad_inst_modal) pad_inst_modal_refresh(); return;
    }
    const uint8_t slot=uint8_t(action/3); const int gesture=action%3;
    if(gesture==1 && s_bank.slot(slot) && s_bank.slot(slot)->type==BT::Action) {
        if(!s_pad_inst_modal) grid_pad_inst_popup_cb(nullptr); // Navigation only; assignment still requires touch confirmation.
    }
    else if(gesture==1) bank_press(slot);
    else if(bank_rotate(slot,gesture==0 ? -1 : 1)) s_bank_highlight[slot]=millis()+500;
}
static lv_obj_t* bank_button(lv_obj_t* parent,int x,int y,int w,int h,const char* text,int action) {
    auto* btn=lv_btn_create(parent); lv_obj_set_pos(btn,x,y); lv_obj_set_size(btn,w,h);
    lv_obj_set_style_pad_all(btn,0,0);
    auto* label=lv_label_create(btn); lv_label_set_text(label,text); lv_obj_center(label);
    lv_obj_add_event_cb(btn,bank_input_touch,LV_EVENT_CLICKED,(void*)(intptr_t)action);
    return btn;
}
static void bank_make_dock() {
    s_bank_dock=lv_obj_create(lv_layer_top());
    lv_obj_set_pos(s_bank_dock,0,0); lv_obj_set_size(s_bank_dock,LCD_H_RES,88);
    lv_obj_set_style_pad_all(s_bank_dock,0,0); lv_obj_set_style_radius(s_bank_dock,0,0);
    lv_obj_set_style_bg_color(s_bank_dock,RED808_PANEL,0); lv_obj_set_style_bg_opa(s_bank_dock,LV_OPA_COVER,0);
    lv_obj_clear_flag(s_bank_dock,LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(s_bank_dock,[](lv_event_t*) { s_bank_dock=nullptr; },LV_EVENT_DELETE,nullptr);
    s_bank_title=lv_label_create(s_bank_dock); lv_obj_set_pos(s_bank_title,8,3);
    lv_obj_set_width(s_bank_title,744); lv_label_set_long_mode(s_bank_title,LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(s_bank_title,&lv_font_montserrat_12,0);
    s_bank_pad_buttons[0]=bank_button(s_bank_dock,760,0,38,23,"P-",-3);
    s_bank_pad_buttons[1]=bank_button(s_bank_dock,800,0,38,23,"P+",-4);
    bank_button(s_bank_dock,844,0,84,23,"BANK <",-1); bank_button(s_bank_dock,930,0,86,23,"BANK >",-2);
    for(uint8_t i=0;i<4;++i) {
        const int x=4+i*254;
        s_bank_cells[i]=bank_button(s_bank_dock,x+34,27,182,56,"",i*3+1);
        s_bank_names[i]=lv_label_create(s_bank_cells[i]); lv_obj_set_width(s_bank_names[i],178);
        lv_obj_set_style_text_align(s_bank_names[i],LV_TEXT_ALIGN_CENTER,0);
        lv_obj_set_style_text_font(s_bank_names[i],&lv_font_montserrat_12,0);
        lv_obj_set_pos(s_bank_names[i],2,3);
        s_bank_values[i]=lv_label_create(s_bank_cells[i]); lv_obj_align(s_bank_values[i],LV_ALIGN_BOTTOM_MID,0,-4);
        lv_obj_set_style_text_font(s_bank_values[i],&lv_font_montserrat_16,0);
        bank_button(s_bank_dock,x,27,32,56,"-",i*3);
        bank_button(s_bank_dock,x+218,27,32,56,"+",i*3+2);
    }
}
// Place existing page coordinates below the rotary strip, including the header.
static void bank_reserve(lv_obj_t* root) {
    for(uint32_t i=0;i<lv_obj_get_child_cnt(root);++i)
        if(lv_obj_get_user_data(lv_obj_get_child(root,i))==&s_bank) return;
    auto* view=lv_obj_create(root); lv_obj_set_user_data(view,&s_bank);
    lv_obj_set_pos(view,0,88); lv_obj_set_size(view,LCD_H_RES,LCD_V_RES-88);
    lv_obj_set_style_pad_all(view,0,0); lv_obj_set_style_border_width(view,0,0);
    lv_obj_set_style_radius(view,0,0); lv_obj_set_style_bg_opa(view,LV_OPA_TRANSP,0);
    lv_obj_set_scroll_dir(view,LV_DIR_VER);
    if(root==scr_live || root==scr_piano || root==scr_piano_params)
        lv_obj_clear_flag(view,LV_OBJ_FLAG_SCROLLABLE);
    while(lv_obj_get_child_cnt(root)>1) {
        auto* child=lv_obj_get_child(root,0);
        if(child==view) child=lv_obj_get_child(root,1);
        lv_obj_set_parent(child,view);
    }
}
static bool bank_blocked(lv_obj_t* allowed) {
    if(s_screensaver_active) return true;
    auto* top=lv_layer_top();
    for(uint32_t i=0;i<lv_obj_get_child_cnt(top);++i) {
        auto* child=lv_obj_get_child(top,i);
        if(child==s_bank_dock || child==allowed || (allowed==s_pad_fx_modal && child==s_pad_inst_modal) || lv_obj_has_flag(child,LV_OBJ_FLAG_HIDDEN)) continue;
        if(lv_obj_get_width(child)>300 && lv_obj_get_height(child)>180 &&
           lv_obj_has_flag(child,LV_OBJ_FLAG_CLICKABLE)) return true;
    }
    return false;
}
static void bank_tick() {
    if(!s_bank_defaults_ready) {
        for(int e=0;e<SP_ENGINE_COUNT;++e) if(!s_pp_initialized[e]) pp_init_engine_defaults(e);
        s_bank_defaults_ready=true;
    }
    for(uint8_t e=0;e<SP_ENGINE_COUNT;++e) {
        const auto& engine=SP_ENGINES[e]; const uint32_t dirty=bank_state::synthDirty[e].exchange(0);
        for(uint8_t i=0;i<engine.param_count;++i) if(dirty&(1u<<engine.params[i].param_id)) {
            s_pp_values[e][i]=bank_state::synth[e][engine.params[i].param_id].load();
            if(lv_scr_act()==scr_piano_params && e==s_pp_engine_idx) pp_cell_redraw_value(i);
        }
    }
    for(uint8_t pad=0;pad<16;++pad) {
        const uint8_t dirty=bank_state::fxDirty[pad].exchange(0);
        if(!dirty) continue;
        auto& st=pad_fx_state_for(pad);
        uint8_t* fields[]={&st.filterType,&st.cutoffU7,&st.resoU7,&st.driveU7,&st.bitsU7,&st.rvbU7,&st.dlyU7};
        for(uint8_t i=0;i<7;++i) if(dirty&(1u<<i)) *fields[i]=uint8_t(bank_state::fxValues[pad][i].load());
        if(s_pad_fx_modal && s_pad_fx_focus_pad==pad) pad_fx_modal_refresh();
    }
    uint8_t count=0; uint32_t kind=0;
    auto* screen=lv_scr_act(); auto* allowed=s_pad_fx_modal ? s_pad_fx_modal : s_pad_inst_modal;
    s_bank_pad=constrain(int(s_pad_fx_modal ? s_pad_fx_focus_pad : s_pad_inst_focus_pad),0,15);
    s_bank_engine=screen==scr_piano_params ? s_pp_engine_idx : s_piano_engine_idx;
    auto add=[&](const char* title,BP a=BP{},BP b=BP{},BP c=BP{},BP d=BP{}) {
        if(count<10) s_banks[count++]={title,{a,b,c,d}};
    };
    const auto level=bp(B_LEVEL,"LEVEL",0,Config::MAX_VOLUME);
    const auto pan=bp(B_PAN,"PAN",-100,100);
    const auto rvb=bp(B_REVERB,"REVERB SEND %",0,100);
    const auto dly=bp(B_DELAY,"DELAY SEND %",0,100);
    const auto vel=bp(B_VELOCITY,"VELOCITY",1,127);
    const auto prob=bp(B_PROB,"PROBABILITY",0,100);
    const auto repeat=bp(B_REPEAT,"REPEAT RATE",0,6,BT::Enum);
    const auto accent=bp(B_ACCENT,"ACCENT",0,1,BT::Toggle);
    const int engine=pad_inst_engine_code(s_pad_inst_sel[s_bank_pad]);
    const bool sampler=engine<0;
    BP pitch=sampler ? bp(B_PITCH,"PITCH st",-24,24) : BP{};
    BP gate=(sampler || engine>=3) ? bp(B_GATE,"GATE ms",0,2000,BT::Continuous,10) : BP{};
    BP source=bp(B_SOURCE,"ENGINE / SAMPLE",0,7,BT::Action); source.flags=bank4::Protected;
    if(!bank_blocked(allowed)) {
        if(s_pad_fx_modal) {
            kind=7; s_bank_pad=s_pad_fx_focus_pad; bank_reserve(s_pad_fx_modal);
            add("FILTER",bp(B_CUTOFF,"CUTOFF",0,127),bp(B_RESO,"RESONANCE",0,127),bp(B_DRIVE,"DRIVE",0,127),bp(B_BITS,"CRUSH bits",4,16,BT::Enum));
            add("SEND",rvb,dly,bp(B_MOD,"CHORUS SEND",0,100));
        } else if(s_pad_inst_modal) {
            kind=4; bank_reserve(s_pad_inst_modal);
            add("BASIC",level,pitch,pan,source);
            if(sampler) add("AMP ENV",bp(B_FADE_IN,"ATTACK ms",0,255),{}, {},bp(B_FADE_OUT,"RELEASE ms",0,255));
            if(engine>=3 && engine<=6) { s_bank_engine=engine-3; bank_append_synth(count,s_bank_engine); }
            add("FILTER",bp(B_CUTOFF,"CUTOFF",0,127),bp(B_RESO,"RESONANCE",0,127),{},bp(B_DRIVE,"DRIVE",0,127));
            add("FX SEND",rvb,dly,bp(B_MOD,"CHORUS SEND",0,100));
            add("PERF",vel,accent,prob,repeat);
        } else if(screen==scr_fx) {
            kind=1; bank_reserve(screen);
            const char* titles[]={"MOD / SPACE","CRUSH / FILTER","COLOR"};
            for(int b=0;b<3;++b) {
                BP slots[4]; for(int i=0;i<4;++i) {
                    const int cell=b*4+i;
                    slots[i]=bp(B_FX+cell,fx_names[cell],cell==9 ? 4 : 0,cell==8 ? 100 : cell==9 ? 16 : cell==11 ? 15 : 127,
                                cell==9 || cell==11 ? BT::Enum : BT::Integer);
                    slots[i].color=fx_colors[cell];
                }
                add(titles[b],slots[0],slots[1],slots[2],slots[3]);
            }
        } else if(screen==scr_volumes) {
            kind=2; bank_reserve(screen); add("CHANNEL",level,pan,rvb,dly);
            add("STATUS",bp(B_MUTE,"MUTE",0,1,BT::Toggle),bp(B_SOLO,"SOLO",0,1,BT::Toggle),source);
        } else if(screen==scr_live) {
            kind=3; bank_reserve(screen); add("PAD",vel,pitch,pan,level); add("PLAY",prob,repeat,gate,accent);
            add("SEND",rvb,dly,bp(B_MOD,"CHORUS SEND",0,100));
        } else if(screen==scr_piano) {
            kind=5; bank_reserve(screen); add("PERFORMANCE",bp(B_OCTAVE,"OCTAVE",1,7),bp(B_PIANO_VEL,"VELOCITY",1,127),
                bp(B_PIANO_ENGINE,"ENGINE",0,3,BT::Enum),bp(B_PIANO_VOL,"VOLUME",0,Config::MAX_VOLUME));
            bank_append_synth(count,s_bank_engine);
        } else if(screen==scr_piano_params) { kind=6; bank_reserve(screen); bank_append_synth(count,s_bank_engine); }
    }
    const uint32_t key=count ? (kind<<24)|(uint32_t(kind==5 || kind==6 || kind==4 ? s_bank_engine : 0)<<16)|(kind==5 || kind==6 || kind==1 ? 0 : (uint32_t(s_bank_pad)<<8)|uint8_t(engine+1)) : 0;
    if(kind==2 || kind==3 || kind==4 || kind==7)
        for(uint8_t b=0;b<count;++b) for(auto& p:s_banks[b].slots)
            p.color=theme_presets[ui_theme_index()].track_colors[s_bank_pad];
    const char* names[]={"","FX LAB","MIXER","LIVE","PAD SOUND","PIANO","PIANO PARAMS","PAD FX"};
    if(kind==5 || kind==6) snprintf(s_bank_context_name,sizeof(s_bank_context_name),"%s %s",names[kind],SP_ENGINES[s_bank_engine].label);
    else if(kind==1) snprintf(s_bank_context_name,sizeof(s_bank_context_name),"FX LAB");
    else snprintf(s_bank_context_name,sizeof(s_bank_context_name),"%s PAD %02d",names[kind],s_bank_pad+1);
    const bool changed=s_bank.setContext({key,s_bank_context_name,s_banks,count,nullptr,bank_get,bank_set});
    if(changed) bank_input::clear();
    if(kind==1 && s_fx_bank_requested>=0) {
        s_bank.select(s_fx_bank_requested); s_fx_bank_requested=-1; bank_input::clear();
    }
    bank_input::active=count!=0;
    if(!count) {
        for(int i=0;i<4;++i) { bank_input::take(i); bank_input::presses[i]=0; bank_input::leds[i]=0; }
        if(s_bank_dock && lv_obj_is_valid(s_bank_dock)) lv_obj_add_flag(s_bank_dock,LV_OBJ_FLAG_HIDDEN);
        return;
    }
    const uint32_t oldRevision=s_bank.revision;
    if(p4_fader_detected()) s_bank.fader(p4_fader_value(),millis());
    if(oldRevision!=s_bank.revision) { bank_input::clear(); ui_note_control_activity(); }
    if(kind==1 && (changed || s_fx_bank_visual!=s_bank.active)) {
        s_fx_bank_visual=s_bank.active;
        fx_page=s_bank.active/(fx_view_modes[fx_view_mode]/4);
        fx_apply_layout();
    }
    for(uint8_t i=0;i<4;++i) {
        const int delta=bank_input::take(i); const uint32_t press=bank_input::takePress(i);
        if(delta) { ui_note_control_activity(); if(bank_rotate(i,delta)) s_bank_highlight[i]=millis()+500; }
        if(press&1) { ui_note_control_activity(); bank_press(i); }
    }
    const bool dockCreated=!s_bank_dock || !lv_obj_is_valid(s_bank_dock);
    if(dockCreated) bank_make_dock();
    static int dockTheme=-1;
    if(dockCreated || dockTheme!=ui_theme_index()) {
        dockTheme=ui_theme_index();
        lv_obj_set_style_bg_color(s_bank_dock,theme_panel(),0);
        lv_obj_set_style_border_color(s_bank_dock,theme_border(),0);
        lv_obj_set_style_text_color(s_bank_title,theme_text(),0);
        for(uint32_t n=0;n<lv_obj_get_child_cnt(s_bank_dock);++n) {
            auto* button=lv_obj_get_child(s_bank_dock,n);
            if(!lv_obj_check_type(button,&lv_btn_class)) continue;
            lv_obj_set_style_bg_color(button,theme_surface(),0);
            lv_obj_set_style_bg_grad_dir(button,LV_GRAD_DIR_NONE,0);
            lv_obj_set_style_bg_opa(button,LV_OPA_COVER,0);
            lv_obj_set_style_text_color(button,theme_text(),0);
            lv_obj_set_style_border_width(button,1,0);
            lv_obj_set_style_border_color(button,theme_border(),0);
            lv_obj_set_style_shadow_width(button,0,0);
            lv_obj_set_style_bg_color(button,lv_color_mix(theme_accent(),theme_panel(),90),LV_STATE_PRESSED);
            lv_obj_set_style_text_color(button,theme_text(),LV_STATE_PRESSED);
        }
    }
    for(auto* button:s_bank_pad_buttons) {
        if(kind==1 || kind==5 || kind==6) lv_obj_add_flag(button,LV_OBJ_FLAG_HIDDEN);
        else lv_obj_clear_flag(button,LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_clear_flag(s_bank_dock,LV_OBJ_FLAG_HIDDEN);
    if(lv_obj_get_child(lv_layer_top(),-1)!=s_bank_dock) lv_obj_move_foreground(s_bank_dock);
    char title[128]; snprintf(title,sizeof(title),"%s  BANK %u/%u  %s",s_bank.context.title,s_bank.active+1,count,s_banks[s_bank.active].title);
    if(strcmp(lv_label_get_text(s_bank_title),title)) lv_label_set_text(s_bank_title,title);
    static uint32_t lastBorder[4]={}, lastContext=0; static uint8_t lastBank=255;
    static float lastValue[4]={};
    const bool sameBank=lastContext==key && lastBank==s_bank.active;
    lastContext=key; lastBank=s_bank.active;
    for(uint8_t i=0;i<4;++i) {
        const auto* p=s_bank.slot(i); char label[64],value[48],unit[24];
        const float currentValue=s_bank.value(i);
        if(sameBank && fabsf(currentValue-lastValue[i])>1e-6f) s_bank_highlight[i]=millis()+500;
        lastValue[i]=currentValue;
        snprintf(label,sizeof(label),"R%u %s%s",i+1,p->label ? p->label : "--",s_bank.fine[i] ? " [F]" : "");
        if(!p->label) snprintf(value,sizeof(value),"--");
        else if(p->type==BT::Action) snprintf(value,sizeof(value),"TOUCH ASSIGN");
        else if(p->id>=B_SYNTH) pp_format_value(value,sizeof(value),synthBankParam(SP_ENGINES[s_bank_engine],p->id-B_SYNTH),s_bank.value(i));
        else if(p->id>=B_FX && fx_card_is_muted(p->id-B_FX)) snprintf(value,sizeof(value),"OFF");
        else if(p->id==B_FX+8) snprintf(value,sizeof(value),"%.0f %%",s_bank.value(i));
        else if(p->id>=B_FX && p->id!=B_FX+9 && p->id!=B_FX+11) {
            fx_format_display_value(p->id-B_FX,int(s_bank.value(i)),fx_card_is_muted(p->id-B_FX),value,sizeof(value),unit,sizeof(unit));
            const size_t used=strlen(value);
            if(unit[0]) snprintf(value+used,sizeof(value)-used," %s",unit);
        }
        else if(p->id==B_FX+11) snprintf(value,sizeof(value),"%s",fx_filter_model_name(int(s_bank.value(i))));
        else if(p->id==B_REVERB || p->id==B_DELAY) snprintf(value,sizeof(value),"%.0f %%",s_bank.value(i));
        else if(p->id==B_CUTOFF) snprintf(value,sizeof(value),"%.0f Hz",pad_fx_cutoff_hz(uint8_t(s_bank.value(i))));
        else if(p->id==B_RESO) snprintf(value,sizeof(value),"%.2f",pad_fx_resonance(uint8_t(s_bank.value(i))));
        else if(p->id==B_REPEAT) snprintf(value,sizeof(value),"%s",NR_LABEL[int(s_bank.value(i))]);
        else if(p->id==B_GATE && s_bank.value(i)==0) snprintf(value,sizeof(value),"AUTO");
        else if(p->id==B_PIANO_ENGINE) snprintf(value,sizeof(value),"%s",PIANO_ENGINE_LABELS[int(s_bank.value(i))]);
        else if(p->type==BT::Toggle) snprintf(value,sizeof(value),"%s",s_bank.value(i)>.5f ? "ON" : "OFF");
        else snprintf(value,sizeof(value),"%.0f",s_bank.value(i));
        if(strcmp(lv_label_get_text(s_bank_names[i]),label)) lv_label_set_text(s_bank_names[i],label);
        if(strcmp(lv_label_get_text(s_bank_values[i]),value)) lv_label_set_text(s_bank_values[i],value);
        const bool lit=int32_t(s_bank_highlight[i]-millis())>0;
        const uint32_t border=lit ? theme_presets[ui_theme_index()].success : p->color;
        if(lastBorder[i]!=border || changed || dockCreated) {
            lv_obj_set_style_border_color(s_bank_cells[i],lv_color_hex(border),0); lastBorder[i]=border;
        }
        bank_input::leds[i]=!p->label || (p->flags&bank4::Disabled) || p->type==BT::Action ? 0 : uint16_t(51+s_bank.normalized(i)*921.f);
    }
}
}
