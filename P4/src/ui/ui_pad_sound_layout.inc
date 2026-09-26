// Included inside grid_pad_inst_popup_cb, after its open/close guard.
    s_pad_sound_manual=false;
    for(int i=0;i<16;++i) {
        s_pad_inst_pending[i]=s_pad_inst_sel[i];
        s_pad_kit_pending[i]=s_pad_kit_assigned[i];
        s_pad_sound_volume[i]=p4.track_volume[i];
        s_pad_sound_preset_dirty[i]=false;
        s_pad_sound_character_dirty[i]=false;
        s_pad_sound_character[i]=s_pad_sound_character_applied[i];
    }
    s_pad_inst_modal=lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_pad_inst_modal,LV_HOR_RES,LV_VER_RES);
    lv_obj_set_style_pad_all(s_pad_inst_modal,0,0);
    lv_obj_set_style_border_width(s_pad_inst_modal,0,0);
    lv_obj_set_style_bg_color(s_pad_inst_modal,theme_bg(),0);
    lv_obj_clear_flag(s_pad_inst_modal,LV_OBJ_FLAG_SCROLLABLE);
    auto text=[&](const char* value,int x,int y,int width) {
        auto* label=lv_label_create(s_pad_inst_modal);
        lv_label_set_text(label,value); lv_obj_set_pos(label,x,y); lv_obj_set_width(label,width);
        lv_obj_set_style_text_color(label,theme_text(),0);
        lv_obj_set_style_text_font(label,&lv_font_montserrat_16,0);
        return label;
    };
    auto button=[&](const char* value,int x,int width,lv_event_cb_t cb) {
        auto* b=lv_btn_create(s_pad_inst_modal); lv_obj_set_pos(b,x,535); lv_obj_set_size(b,width,48);
        apply_control_button_style(b,theme_border(),false,8);
        auto* label=lv_label_create(b); lv_label_set_text(label,value); lv_obj_center(label);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,nullptr); return label;
    };
    s_pad_inst_modal_pad_lbl=text("PAD SOUND",24,84,210);
    s_pad_inst_modal_inst_lbl=text("",250,84,260);
    text("Toca un pad y gira R1-R4. ASIGNAR confirma.",540,84,460);
    static const char* padNames[]={"BD","SD","CH","OH","CY","CP","RS","CB","LT","MT","HT","MA","CL","HC","MC","LC"};
    for(int i=0;i<16;++i) {
        auto* b=lv_btn_create(s_pad_inst_modal); s_pad_inst_modal_pad_btns[i]=b;
        lv_obj_set_pos(b,24+(i%4)*124,132+(i/4)*96); lv_obj_set_size(b,114,86);
        apply_control_button_style(b,ui_track_color(i),false,10);
        auto* label=lv_label_create(b); lv_label_set_text_fmt(label,"%02d\n%s",i+1,padNames[i]);
        lv_obj_set_style_text_align(label,LV_TEXT_ALIGN_CENTER,0); lv_obj_center(label);
        lv_obj_add_event_cb(b,pad_inst_modal_pick_pad_cb,LV_EVENT_CLICKED,(void*)(intptr_t)i);
    }
    for(int i=0;i<4;++i) {
        auto* panel=lv_obj_create(s_pad_inst_modal);
        lv_obj_set_pos(panel,540,132+i*86); lv_obj_set_size(panel,460,76);
        lv_obj_set_style_bg_color(panel,theme_panel(),0); lv_obj_set_style_border_color(panel,theme_border(),0);
        lv_obj_clear_flag(panel,LV_OBJ_FLAG_SCROLLABLE|LV_OBJ_FLAG_CLICKABLE);
        auto* label=lv_label_create(panel); s_pad_sound_readouts[i]=label;
        lv_obj_set_width(label,420); lv_obj_set_style_text_color(label,theme_text(),0);
        lv_obj_set_style_text_font(label,&lv_font_montserrat_16,0); lv_obj_center(label);
    }
    text("Manual: fader = banco. Parametros del engine compartidos.",540,482,460);
    button("BACK",24,100,pad_inst_modal_close_cb);
    s_pad_sound_mode_label=button("MANUAL",136,150,[](lv_event_t*) {
        if(!s_pad_sound_manual) pad_sound_apply_draft(s_pad_inst_focus_pad);
        s_pad_sound_manual=!s_pad_sound_manual;
        lv_label_set_text(s_pad_sound_mode_label,s_pad_sound_manual ? "SELECCION" : "MANUAL");
        bank_input::clear();
    });
    button("SAMPLER",298,140,pad_inst_sampler_original_cb);
    button("FX",450,100,pad_fx_modal_show);
    button("PREVIEW",562,160,pad_inst_modal_preview_cb);
    button("ASIGNAR",734,180,pad_inst_modal_assign_cb);
    pad_inst_modal_refresh();
