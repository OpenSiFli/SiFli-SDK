enum {
	LVOBJ_create=1,
	LVOBJ_set_obj,
	LVOBJ_bind,
	LVOBJ_set_event_cb,
	LVOBJ_align_to,
	LVOBJ_get_x,
	LVOBJ_get_y,
	LVOBJ_set_local_font,
	LVOBJ_set_page_glue,
	LVOBJ_align,
	LVOBJ_set_pos,
	LVOBJ_move_foreground,
	LVOBJ_get_height,
	LVOBJ_add_flag,
	LVOBJ_set_size,
	LVOBJ_move_background,
	LVOBJ_get_width,
	LVOBJ_delete,
}JS_lv_obj_methods_enum;
const JS_lv_protos lv_obj_protos[]={
	{
		.name="create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="set_obj",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_none
	},
	{
		.name="bind",
		.param_type={LVTYPE_int,LVTYPE_func},
		.return_type=LVTYPE_none
	},
	{
		.name="set_event_cb",
		.param_type={LVTYPE_func},
		.return_type=LVTYPE_none
	},
	{
		.name="align_to",
		.param_type={LVTYPE_object,LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="get_x",
		.return_type=LVTYPE_int
	},
	{
		.name="get_y",
		.return_type=LVTYPE_int
	},
	{
		.name="set_local_font",
		.param_type={LVTYPE_int,LVTYPE_color,},
		.return_type=LVTYPE_none
	},
	{
		.name="set_page_glue",
		.param_type={LVTYPE_bool,},
		.return_type=LVTYPE_none
	},
	{
		.name="align",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="set_pos",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="move_foreground",
		.return_type=LVTYPE_none
	},
	{
		.name="get_height",
		.return_type=LVTYPE_int
	},
	{
		.name="add_flag",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="set_size",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="move_background",
		.return_type=LVTYPE_none
	},
	{
		.name="get_width",
		.return_type=LVTYPE_int
	},
	{
		.name="delete",
		.return_type=LVTYPE_none
	},
};
const JSCFunctionListEntry js_lv_obj_methods[]={
	JS_CFUNC_MAGIC_DEF("create", 1, js_lv_obj_method,LVOBJ_create),
	JS_CFUNC_MAGIC_DEF("set_obj", 1, js_lv_obj_method,LVOBJ_set_obj),
	JS_CFUNC_MAGIC_DEF("bind", 2, js_lv_obj_method,LVOBJ_bind),
	JS_CFUNC_MAGIC_DEF("set_event_cb", 1, js_lv_obj_method,LVOBJ_set_event_cb),
	JS_CFUNC_MAGIC_DEF("align_to", 4, js_lv_obj_method,LVOBJ_align_to),
	JS_CFUNC_MAGIC_DEF("get_x", 0, js_lv_obj_method,LVOBJ_get_x),
	JS_CFUNC_MAGIC_DEF("get_y", 0, js_lv_obj_method,LVOBJ_get_y),
	JS_CFUNC_MAGIC_DEF("set_local_font", 2, js_lv_obj_method,LVOBJ_set_local_font),
	JS_CFUNC_MAGIC_DEF("set_page_glue", 1, js_lv_obj_method,LVOBJ_set_page_glue),
	JS_CFUNC_MAGIC_DEF("align", 3, js_lv_obj_method,LVOBJ_align),
	JS_CFUNC_MAGIC_DEF("set_pos", 2, js_lv_obj_method,LVOBJ_set_pos),
	JS_CFUNC_MAGIC_DEF("move_foreground", 0, js_lv_obj_method,LVOBJ_move_foreground),
	JS_CFUNC_MAGIC_DEF("get_height", 0, js_lv_obj_method,LVOBJ_get_height),
	JS_CFUNC_MAGIC_DEF("add_flag", 1, js_lv_obj_method,LVOBJ_add_flag),
	JS_CFUNC_MAGIC_DEF("set_size", 2, js_lv_obj_method,LVOBJ_set_size),
	JS_CFUNC_MAGIC_DEF("move_background", 0, js_lv_obj_method,LVOBJ_move_background),
	JS_CFUNC_MAGIC_DEF("get_width", 0, js_lv_obj_method,LVOBJ_get_width),
	JS_CFUNC_MAGIC_DEF("delete", 0, js_lv_obj_method,LVOBJ_delete),
};
static JSValue  lv_obj_call_method(JSContext *ctx,JSValueConst this_val, int magic, JSValueConst param[])
{
	JSValue  r=JS_UNDEFINED;
	JSlvobj *s = JS_GetOpaque2(ctx, this_val, js_lvobj_class_id);
	switch (magic) {
		case LVOBJ_create:
		{
			s->lvobj = lv_obj_create((lv_obj_t*)(int)param[0]);
			int r_in=(int)s->lvobj;
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVOBJ_set_obj:
		{
			s->lvobj = (lv_obj_t*)(int)param[0];
			break;
		}
		case LVOBJ_bind:
		{
			if (s->data_cbk)
				JS_FreeValue(ctx, s->data_cbk);
			s->data_cbk=JS_DupValue(ctx, (JSValueConst)param[1]);
			s->data_idx=(int)param[0];
			s->ctx=ctx;
			s->this_obj=this_val;
			lv_data_bind(s);
			break;
		}
		case LVOBJ_set_event_cb:
		{
			if (s->callback)
				JS_FreeValue(ctx, s->callback);
			s->callback=JS_DupValue(ctx, (JSValueConst)param[0]);
			s->ctx=ctx;
			s->this_obj=this_val;
			lv_obj_set_user_data(s->lvobj,s);
			lv_obj_add_event_cb(s->lvobj, lv_event_callback_proxy, LV_EVENT_ALL, NULL);
			break;
		}
		case LVOBJ_align_to:
		{
			lv_obj_align_to(s->lvobj,(lv_obj_t *)param[0],(int)param[1],(int)param[2],(int)param[3]);
			break;
		}
		case LVOBJ_get_x:
		{
			int r_in = (int)lv_obj_get_x(s->lvobj);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVOBJ_get_y:
		{
			int r_in = (int)lv_obj_get_y(s->lvobj);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVOBJ_set_local_font:
		{
			lv_obj_set_local_font(s->lvobj,(int)param[0],lv_color_from_int((int)param[1]));
			break;
		}
		case LVOBJ_set_page_glue:
		{
			lv_obj_set_page_glue(s->lvobj,(bool)param[0]);
			break;
		}
		case LVOBJ_align:
		{
			lv_obj_align(s->lvobj,(int)param[0],(int)param[1],(int)param[2]);
			break;
		}
		case LVOBJ_set_pos:
		{
			lv_obj_set_pos(s->lvobj,(int)param[0],(int)param[1]);
			break;
		}
		case LVOBJ_move_foreground:
		{
			lv_obj_move_foreground(s->lvobj);
			break;
		}
		case LVOBJ_get_height:
		{
			int r_in = (int)lv_obj_get_height(s->lvobj);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVOBJ_add_flag:
		{
			lv_obj_add_flag(s->lvobj,(int)param[0]);
			break;
		}
		case LVOBJ_set_size:
		{
			lv_obj_set_size(s->lvobj,(int)param[0],(int)param[1]);
			break;
		}
		case LVOBJ_move_background:
		{
			lv_obj_move_background(s->lvobj);
			break;
		}
		case LVOBJ_get_width:
		{
			int r_in = (int)lv_obj_get_width(s->lvobj);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVOBJ_delete:
		{
			lv_obj_del(s->lvobj);
			break;
		}
	}
	return r;
}
enum {
	LVFUNC_color_make=1,
	LVFUNC_get_hor_max,
	LVFUNC_get_ver_max,
	LVFUNC_gui_app_cleanup,
	LVFUNC_gui_app_cleanup_now,
	LVFUNC_gui_app_exit,
	LVFUNC_gui_app_get_clock_parent,
	LVFUNC_gui_app_get_running_apps,
	LVFUNC_gui_app_goback,
	LVFUNC_gui_app_goback_to_page,
	LVFUNC_gui_app_init,
	LVFUNC_gui_app_is_actived,
	LVFUNC_gui_app_is_all_closed,
	LVFUNC_gui_app_manual_goback_anim,
	LVFUNC_gui_app_remove_page,
	LVFUNC_gui_app_run,
	LVFUNC_gui_app_run_now,
	LVFUNC_gui_app_self_exit,
	LVFUNC_lvsf_font_height,
	LVFUNC_lvsf_font_width,
	LVFUNC_scr_act,
	LVFUNC_trigo_sin,
}JS_lv_funcs_enum;
const JS_lv_protos lv_funcs_proto[] = {
	{
		.name="color_make",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_color
	},
	{
		.name="get_hor_max",
		.return_type=LVTYPE_int
	},
	{
		.name="get_ver_max",
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_cleanup",
		.return_type=LVTYPE_none
	},
	{
		.name="gui_app_cleanup_now",
		.return_type=LVTYPE_none
	},
	{
		.name="gui_app_exit",
		.param_type={LVTYPE_string,},
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_get_clock_parent",
		.return_type=LVTYPE_object
	},
	{
		.name="gui_app_get_running_apps",
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_goback",
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_goback_to_page",
		.param_type={LVTYPE_string,},
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_init",
		.return_type=LVTYPE_none
	},
	{
		.name="gui_app_is_actived",
		.param_type={LVTYPE_string,},
		.return_type=LVTYPE_bool
	},
	{
		.name="gui_app_is_all_closed",
		.return_type=LVTYPE_bool
	},
	{
		.name="gui_app_manual_goback_anim",
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_remove_page",
		.param_type={LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="gui_app_run",
		.param_type={LVTYPE_string,},
		.return_type=LVTYPE_int
	},
	{
		.name="gui_app_run_now",
		.param_type={LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="gui_app_self_exit",
		.return_type=LVTYPE_none
	},
	{
		.name="lvsf_font_height",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="lvsf_font_width",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="scr_act",
		.return_type=LVTYPE_object
	},
	{
		.name="trigo_sin",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
};
const JSCFunctionListEntry js_lv_funcs[]={
	JS_CFUNC_MAGIC_DEF("color_make", 3, js_lv_func,LVFUNC_color_make),
	JS_CFUNC_MAGIC_DEF("get_hor_max", 0, js_lv_func,LVFUNC_get_hor_max),
	JS_CFUNC_MAGIC_DEF("get_ver_max", 0, js_lv_func,LVFUNC_get_ver_max),
	JS_CFUNC_MAGIC_DEF("gui_app_cleanup", 0, js_lv_func,LVFUNC_gui_app_cleanup),
	JS_CFUNC_MAGIC_DEF("gui_app_cleanup_now", 0, js_lv_func,LVFUNC_gui_app_cleanup_now),
	JS_CFUNC_MAGIC_DEF("gui_app_exit", 1, js_lv_func,LVFUNC_gui_app_exit),
	JS_CFUNC_MAGIC_DEF("gui_app_get_clock_parent", 0, js_lv_func,LVFUNC_gui_app_get_clock_parent),
	JS_CFUNC_MAGIC_DEF("gui_app_get_running_apps", 0, js_lv_func,LVFUNC_gui_app_get_running_apps),
	JS_CFUNC_MAGIC_DEF("gui_app_goback", 0, js_lv_func,LVFUNC_gui_app_goback),
	JS_CFUNC_MAGIC_DEF("gui_app_goback_to_page", 1, js_lv_func,LVFUNC_gui_app_goback_to_page),
	JS_CFUNC_MAGIC_DEF("gui_app_init", 0, js_lv_func,LVFUNC_gui_app_init),
	JS_CFUNC_MAGIC_DEF("gui_app_is_actived", 1, js_lv_func,LVFUNC_gui_app_is_actived),
	JS_CFUNC_MAGIC_DEF("gui_app_is_all_closed", 0, js_lv_func,LVFUNC_gui_app_is_all_closed),
	JS_CFUNC_MAGIC_DEF("gui_app_manual_goback_anim", 0, js_lv_func,LVFUNC_gui_app_manual_goback_anim),
	JS_CFUNC_MAGIC_DEF("gui_app_remove_page", 1, js_lv_func,LVFUNC_gui_app_remove_page),
	JS_CFUNC_MAGIC_DEF("gui_app_run", 1, js_lv_func,LVFUNC_gui_app_run),
	JS_CFUNC_MAGIC_DEF("gui_app_run_now", 1, js_lv_func,LVFUNC_gui_app_run_now),
	JS_CFUNC_MAGIC_DEF("gui_app_self_exit", 0, js_lv_func,LVFUNC_gui_app_self_exit),
	JS_CFUNC_MAGIC_DEF("lvsf_font_height", 1, js_lv_func,LVFUNC_lvsf_font_height),
	JS_CFUNC_MAGIC_DEF("lvsf_font_width", 2, js_lv_func,LVFUNC_lvsf_font_width),
	JS_CFUNC_MAGIC_DEF("scr_act", 0, js_lv_func,LVFUNC_scr_act),
	JS_CFUNC_MAGIC_DEF("trigo_sin", 1, js_lv_func,LVFUNC_trigo_sin),
};
static JSValue  lv_call_func(JSContext *ctx, int magic, JSValueConst param[])
{
	JSValue  r=JS_UNDEFINED;
	switch (magic) {
		case LVFUNC_color_make:
		{
#ifdef DISABLE_LVGL_V8
			int32_t t;
			t = (int)param[2];
			t <<= 8;
			t += (int)param[1];
			t <<= 8;
			t += (int)param[0];
			r = JS_NewInt32(ctx, t);
#else
			lv_color32_t r_in;
			r_in.full=0;
			r_in.ch.red=(int)param[0];
			r_in.ch.green=(int)param[1];
			r_in.ch.blue=(int)param[2];
			r=JS_NewInt32(ctx,r_in.full);
#endif
			break;
		};
		case LVFUNC_get_hor_max:
		{
			int r_in = lv_get_hor_max();
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_get_ver_max:
		{
			int r_in = lv_get_ver_max();
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_cleanup:
		{
			gui_app_cleanup();
			break;
		}
		case LVFUNC_gui_app_cleanup_now:
		{
			gui_app_cleanup_now();
			break;
		}
		case LVFUNC_gui_app_exit:
		{
			int r_in = gui_app_exit((char *)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_get_clock_parent:
		{
			lv_obj_t * r_in = gui_app_get_clock_parent();
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_gui_app_get_running_apps:
		{
			int r_in = gui_app_get_running_apps();
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_goback:
		{
			int r_in = gui_app_goback();
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_goback_to_page:
		{
			int r_in = gui_app_goback_to_page((char *)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_init:
		{
			gui_app_init();
			break;
		}
		case LVFUNC_gui_app_is_actived:
		{
			bool r_in = gui_app_is_actived((char *)param[0]);
			r=JS_NewBool(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_is_all_closed:
		{
			bool r_in = gui_app_is_all_closed();
			r=JS_NewBool(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_manual_goback_anim:
		{
			int r_in = gui_app_manual_goback_anim();
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_remove_page:
		{
			gui_app_remove_page((char *)param[0]);
			break;
		}
		case LVFUNC_gui_app_run:
		{
			int r_in = gui_app_run((char *)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gui_app_run_now:
		{
			gui_app_run_now((char *)param[0]);
			break;
		}
		case LVFUNC_gui_app_self_exit:
		{
			gui_app_self_exit();
			break;
		}
		case LVFUNC_lvsf_font_height:
		{
			int r_in = lvsf_font_height((int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_lvsf_font_width:
		{
			int r_in = lvsf_font_width((int)param[0],(int)param[1]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_scr_act:
		{
			lv_obj_t * r_in = lv_scr_act();
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_trigo_sin:
		{
			int r_in = lv_trigo_sin((int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
	}
	return r;
}
enum {
	LVFUNC_lvsfcomp_create=1,
	LVFUNC_lvsfcomp_img,
	LVFUNC_lvsfcomp_set_ring,
	LVFUNC_lvsfcomp_set_type,
	LVFUNC_lvsfcomp_arc,
	LVFUNC_lvsfcomp_set_arc,
	LVFUNC_lvsfcomp_text,
	LVFUNC_lvsfcomp_ring,
	LVFUNC_qrcode_create,
	LVFUNC_qrcode_set_text,
	LVFUNC_qrcode_setparam,
	LVFUNC_lvsfcorner_create,
	LVFUNC_lvsfcorner_curve_text,
	LVFUNC_lvsfcorner_zone,
	LVFUNC_lvsfcorner_img,
	LVFUNC_lvsfcorner_text,
	LVFUNC_lvsfcorner_arc,
	LVFUNC_lvsfcorner_arc_scale,
	LVFUNC_analogclk_create,
	LVFUNC_analogclk_pos_off,
	LVFUNC_analogclk_img,
	LVFUNC_analogclk_refr_inteval,
	LVFUNC_img_create,
	LVFUNC_img_set_antialias,
	LVFUNC_img_set_offset_y,
	LVFUNC_img_set_offset_x,
	LVFUNC_img_set_zoom,
	LVFUNC_img_set_size_mode,
	LVFUNC_img_set_angle,
	LVFUNC_img_get_size_mode,
	LVFUNC_img_set_src,
	LVFUNC_img_get_offset_y,
	LVFUNC_img_get_offset_x,
	LVFUNC_img_get_zoom,
	LVFUNC_img_get_angle,
	LVFUNC_img_decoder_create,
	LVFUNC_img_set_pivot,
	LVFUNC_img_get_antialias,
	LVFUNC_lvsfcurve_create,
	LVFUNC_lvsfcurve_set_buf,
	LVFUNC_lvsfcurve_draw_arc,
	LVFUNC_lvsfcurve_set_pivot,
	LVFUNC_lvsfcurve_text,
	LVFUNC_lvsfezipa_create,
	LVFUNC_lvsfezipa_stop,
	LVFUNC_lvsfezipa_play,
	LVFUNC_lvsfezipa_set_src,
	LVFUNC_label_create,
	LVFUNC_label_set_text_static,
	LVFUNC_label_get_text,
	LVFUNC_label_get_text_selection_start,
	LVFUNC_label_get_recolor,
	LVFUNC_label_ins_text,
	LVFUNC_label_set_text_sel_end,
	LVFUNC_label_set_long_mode,
	LVFUNC_label_set_text_sel_start,
	LVFUNC_label_get_long_mode,
	LVFUNC_label_set_text,
	LVFUNC_label_cut_text,
	LVFUNC_label_get_text_selection_end,
	LVFUNC_label_set_recolor,
	LVFUNC_gif_create,
	LVFUNC_gif_pause,
	LVFUNC_gif_set_src,
	LVFUNC_gif_restart,
	LVFUNC_gif_resume,
	LVFUNC_lvsfbarcode_create,
	LVFUNC_lvsfbarcode_set_text,
	LVFUNC_idximg_create,
	LVFUNC_idximg_prefix,
	LVFUNC_idximg_select,
	LVFUNC_rlottie_create,
	LVFUNC_rlottie_raw,
	LVFUNC_rlottie_play,
	LVFUNC_rlottie_file,
}JS_lv_ext_funcs_enum;
const JS_lv_protos lv_ext_funcs_proto[] = {
	{
		.name="lvsfcomp_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="lvsfcomp_img",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcomp_set_ring",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcomp_set_type",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcomp_arc",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_color,LVTYPE_color,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcomp_set_arc",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcomp_text",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcomp_ring",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_color,LVTYPE_color,},
		.return_type=LVTYPE_object
	},
	{
		.name="qrcode_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="qrcode_set_text",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="qrcode_setparam",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_color,LVTYPE_color,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcorner_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="lvsfcorner_curve_text",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcorner_zone",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcorner_img",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcorner_text",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcorner_arc",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_color,},
		.return_type=LVTYPE_object
	},
	{
		.name="lvsfcorner_arc_scale",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="analogclk_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="analogclk_pos_off",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="analogclk_img",
		.param_type={LVTYPE_int,LVTYPE_string,LVTYPE_string,LVTYPE_string,LVTYPE_string,},
		.return_type=LVTYPE_int
	},
	{
		.name="analogclk_refr_inteval",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="img_set_antialias",
		.param_type={LVTYPE_int,LVTYPE_bool,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_set_offset_y",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_set_offset_x",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_set_zoom",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_set_size_mode",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_set_angle",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_get_size_mode",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="img_set_src",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_get_offset_y",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="img_get_offset_x",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="img_get_zoom",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="img_get_angle",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="img_decoder_create",
		.return_type=LVTYPE_none
	},
	{
		.name="img_set_pivot",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="img_get_antialias",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_bool
	},
	{
		.name="lvsfcurve_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="lvsfcurve_set_buf",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcurve_draw_arc",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_int,LVTYPE_color,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcurve_set_pivot",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfcurve_text",
		.param_type={LVTYPE_int,LVTYPE_string,LVTYPE_int,LVTYPE_int,LVTYPE_color,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfezipa_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="lvsfezipa_stop",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfezipa_play",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfezipa_set_src",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="label_set_text_static",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_get_text",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_string
	},
	{
		.name="label_get_text_selection_start",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="label_get_recolor",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_bool
	},
	{
		.name="label_ins_text",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_set_text_sel_end",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_set_long_mode",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_set_text_sel_start",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_get_long_mode",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="label_set_text",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_cut_text",
		.param_type={LVTYPE_int,LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="label_get_text_selection_end",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="label_set_recolor",
		.param_type={LVTYPE_int,LVTYPE_bool,},
		.return_type=LVTYPE_none
	},
	{
		.name="gif_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="gif_pause",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="gif_set_src",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="gif_restart",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="gif_resume",
		.param_type={LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="lvsfbarcode_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="lvsfbarcode_set_text",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="idximg_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="idximg_prefix",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_none
	},
	{
		.name="idximg_select",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_none
	},
	{
		.name="rlottie_create",
		.param_type={LVTYPE_int},
		.return_type=LVTYPE_int
	},
	{
		.name="rlottie_raw",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_int
	},
	{
		.name="rlottie_play",
		.param_type={LVTYPE_int,LVTYPE_int,},
		.return_type=LVTYPE_int
	},
	{
		.name="rlottie_file",
		.param_type={LVTYPE_int,LVTYPE_string,},
		.return_type=LVTYPE_int
	},
};
const JSCFunctionListEntry js_lv_ext_funcs[]={
	JS_CFUNC_MAGIC_DEF("lvsfcomp_create", 2, js_lv_ext_func,LVFUNC_lvsfcomp_create),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_img", 2, js_lv_ext_func,LVFUNC_lvsfcomp_img),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_set_ring", 3, js_lv_ext_func,LVFUNC_lvsfcomp_set_ring),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_set_type", 2, js_lv_ext_func,LVFUNC_lvsfcomp_set_type),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_arc", 4, js_lv_ext_func,LVFUNC_lvsfcomp_arc),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_set_arc", 2, js_lv_ext_func,LVFUNC_lvsfcomp_set_arc),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_text", 2, js_lv_ext_func,LVFUNC_lvsfcomp_text),
	JS_CFUNC_MAGIC_DEF("lvsfcomp_ring", 5, js_lv_ext_func,LVFUNC_lvsfcomp_ring),
	JS_CFUNC_MAGIC_DEF("qrcode_create", 2, js_lv_ext_func,LVFUNC_qrcode_create),
	JS_CFUNC_MAGIC_DEF("qrcode_set_text", 2, js_lv_ext_func,LVFUNC_qrcode_set_text),
	JS_CFUNC_MAGIC_DEF("qrcode_setparam", 4, js_lv_ext_func,LVFUNC_qrcode_setparam),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_create", 2, js_lv_ext_func,LVFUNC_lvsfcorner_create),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_curve_text", 2, js_lv_ext_func,LVFUNC_lvsfcorner_curve_text),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_zone", 5, js_lv_ext_func,LVFUNC_lvsfcorner_zone),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_img", 2, js_lv_ext_func,LVFUNC_lvsfcorner_img),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_text", 2, js_lv_ext_func,LVFUNC_lvsfcorner_text),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_arc", 4, js_lv_ext_func,LVFUNC_lvsfcorner_arc),
	JS_CFUNC_MAGIC_DEF("lvsfcorner_arc_scale", 2, js_lv_ext_func,LVFUNC_lvsfcorner_arc_scale),
	JS_CFUNC_MAGIC_DEF("analogclk_create", 2, js_lv_ext_func,LVFUNC_analogclk_create),
	JS_CFUNC_MAGIC_DEF("analogclk_pos_off", 4, js_lv_ext_func,LVFUNC_analogclk_pos_off),
	JS_CFUNC_MAGIC_DEF("analogclk_img", 5, js_lv_ext_func,LVFUNC_analogclk_img),
	JS_CFUNC_MAGIC_DEF("analogclk_refr_inteval", 2, js_lv_ext_func,LVFUNC_analogclk_refr_inteval),
	JS_CFUNC_MAGIC_DEF("img_create", 2, js_lv_ext_func,LVFUNC_img_create),
	JS_CFUNC_MAGIC_DEF("img_set_antialias", 2, js_lv_ext_func,LVFUNC_img_set_antialias),
	JS_CFUNC_MAGIC_DEF("img_set_offset_y", 2, js_lv_ext_func,LVFUNC_img_set_offset_y),
	JS_CFUNC_MAGIC_DEF("img_set_offset_x", 2, js_lv_ext_func,LVFUNC_img_set_offset_x),
	JS_CFUNC_MAGIC_DEF("img_set_zoom", 2, js_lv_ext_func,LVFUNC_img_set_zoom),
	JS_CFUNC_MAGIC_DEF("img_set_size_mode", 2, js_lv_ext_func,LVFUNC_img_set_size_mode),
	JS_CFUNC_MAGIC_DEF("img_set_angle", 2, js_lv_ext_func,LVFUNC_img_set_angle),
	JS_CFUNC_MAGIC_DEF("img_get_size_mode", 1, js_lv_ext_func,LVFUNC_img_get_size_mode),
	JS_CFUNC_MAGIC_DEF("img_set_src", 2, js_lv_ext_func,LVFUNC_img_set_src),
	JS_CFUNC_MAGIC_DEF("img_get_offset_y", 1, js_lv_ext_func,LVFUNC_img_get_offset_y),
	JS_CFUNC_MAGIC_DEF("img_get_offset_x", 1, js_lv_ext_func,LVFUNC_img_get_offset_x),
	JS_CFUNC_MAGIC_DEF("img_get_zoom", 1, js_lv_ext_func,LVFUNC_img_get_zoom),
	JS_CFUNC_MAGIC_DEF("img_get_angle", 1, js_lv_ext_func,LVFUNC_img_get_angle),
	JS_CFUNC_MAGIC_DEF("img_decoder_create", 0, js_lv_ext_func,LVFUNC_img_decoder_create),
	JS_CFUNC_MAGIC_DEF("img_set_pivot", 3, js_lv_ext_func,LVFUNC_img_set_pivot),
	JS_CFUNC_MAGIC_DEF("img_get_antialias", 1, js_lv_ext_func,LVFUNC_img_get_antialias),
	JS_CFUNC_MAGIC_DEF("lvsfcurve_create", 2, js_lv_ext_func,LVFUNC_lvsfcurve_create),
	JS_CFUNC_MAGIC_DEF("lvsfcurve_set_buf", 3, js_lv_ext_func,LVFUNC_lvsfcurve_set_buf),
	JS_CFUNC_MAGIC_DEF("lvsfcurve_draw_arc", 6, js_lv_ext_func,LVFUNC_lvsfcurve_draw_arc),
	JS_CFUNC_MAGIC_DEF("lvsfcurve_set_pivot", 3, js_lv_ext_func,LVFUNC_lvsfcurve_set_pivot),
	JS_CFUNC_MAGIC_DEF("lvsfcurve_text", 6, js_lv_ext_func,LVFUNC_lvsfcurve_text),
	JS_CFUNC_MAGIC_DEF("lvsfezipa_create", 2, js_lv_ext_func,LVFUNC_lvsfezipa_create),
	JS_CFUNC_MAGIC_DEF("lvsfezipa_stop", 1, js_lv_ext_func,LVFUNC_lvsfezipa_stop),
	JS_CFUNC_MAGIC_DEF("lvsfezipa_play", 1, js_lv_ext_func,LVFUNC_lvsfezipa_play),
	JS_CFUNC_MAGIC_DEF("lvsfezipa_set_src", 2, js_lv_ext_func,LVFUNC_lvsfezipa_set_src),
	JS_CFUNC_MAGIC_DEF("label_create", 2, js_lv_ext_func,LVFUNC_label_create),
	JS_CFUNC_MAGIC_DEF("label_set_text_static", 2, js_lv_ext_func,LVFUNC_label_set_text_static),
	JS_CFUNC_MAGIC_DEF("label_get_text", 1, js_lv_ext_func,LVFUNC_label_get_text),
	JS_CFUNC_MAGIC_DEF("label_get_text_selection_start", 1, js_lv_ext_func,LVFUNC_label_get_text_selection_start),
	JS_CFUNC_MAGIC_DEF("label_get_recolor", 1, js_lv_ext_func,LVFUNC_label_get_recolor),
	JS_CFUNC_MAGIC_DEF("label_ins_text", 3, js_lv_ext_func,LVFUNC_label_ins_text),
	JS_CFUNC_MAGIC_DEF("label_set_text_sel_end", 2, js_lv_ext_func,LVFUNC_label_set_text_sel_end),
	JS_CFUNC_MAGIC_DEF("label_set_long_mode", 2, js_lv_ext_func,LVFUNC_label_set_long_mode),
	JS_CFUNC_MAGIC_DEF("label_set_text_sel_start", 2, js_lv_ext_func,LVFUNC_label_set_text_sel_start),
	JS_CFUNC_MAGIC_DEF("label_get_long_mode", 1, js_lv_ext_func,LVFUNC_label_get_long_mode),
	JS_CFUNC_MAGIC_DEF("label_set_text", 2, js_lv_ext_func,LVFUNC_label_set_text),
	JS_CFUNC_MAGIC_DEF("label_cut_text", 3, js_lv_ext_func,LVFUNC_label_cut_text),
	JS_CFUNC_MAGIC_DEF("label_get_text_selection_end", 1, js_lv_ext_func,LVFUNC_label_get_text_selection_end),
	JS_CFUNC_MAGIC_DEF("label_set_recolor", 2, js_lv_ext_func,LVFUNC_label_set_recolor),
	JS_CFUNC_MAGIC_DEF("gif_create", 2, js_lv_ext_func,LVFUNC_gif_create),
	JS_CFUNC_MAGIC_DEF("gif_pause", 1, js_lv_ext_func,LVFUNC_gif_pause),
	JS_CFUNC_MAGIC_DEF("gif_set_src", 2, js_lv_ext_func,LVFUNC_gif_set_src),
	JS_CFUNC_MAGIC_DEF("gif_restart", 1, js_lv_ext_func,LVFUNC_gif_restart),
	JS_CFUNC_MAGIC_DEF("gif_resume", 1, js_lv_ext_func,LVFUNC_gif_resume),
	JS_CFUNC_MAGIC_DEF("lvsfbarcode_create", 2, js_lv_ext_func,LVFUNC_lvsfbarcode_create),
	JS_CFUNC_MAGIC_DEF("lvsfbarcode_set_text", 2, js_lv_ext_func,LVFUNC_lvsfbarcode_set_text),
	JS_CFUNC_MAGIC_DEF("idximg_create", 2, js_lv_ext_func,LVFUNC_idximg_create),
	JS_CFUNC_MAGIC_DEF("idximg_prefix", 2, js_lv_ext_func,LVFUNC_idximg_prefix),
	JS_CFUNC_MAGIC_DEF("idximg_select", 2, js_lv_ext_func,LVFUNC_idximg_select),
	JS_CFUNC_MAGIC_DEF("rlottie_create", 2, js_lv_ext_func,LVFUNC_rlottie_create),
	JS_CFUNC_MAGIC_DEF("rlottie_raw", 2, js_lv_ext_func,LVFUNC_rlottie_raw),
	JS_CFUNC_MAGIC_DEF("rlottie_play", 2, js_lv_ext_func,LVFUNC_rlottie_play),
	JS_CFUNC_MAGIC_DEF("rlottie_file", 2, js_lv_ext_func,LVFUNC_rlottie_file),
};
static JSValue  lv_ext_call_func(JSContext *ctx, int magic, JSValueConst param[])
{
	JSValue  r=JS_UNDEFINED;
	switch (magic) {
		case LVFUNC_lvsfcomp_create:
		{
			int r_in = (int)lv_lvsfcomp_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_lvsfcomp_img:
		{
			lv_obj_t * r_in = lv_lvsfcomp_img((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcomp_set_ring:
		{
			lv_lvsfcomp_set_ring((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2]);
			break;
		}
		case LVFUNC_lvsfcomp_set_type:
		{
			lv_lvsfcomp_set_type((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_lvsfcomp_arc:
		{
			lv_obj_t * r_in = lv_lvsfcomp_arc((lv_obj_t*)((int)param[0]),(int)param[1],lv_color_from_int((int)param[2]),lv_color_from_int((int)param[3]));
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcomp_set_arc:
		{
			lv_lvsfcomp_set_arc((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_lvsfcomp_text:
		{
			lv_obj_t * r_in = lv_lvsfcomp_text((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcomp_ring:
		{
			lv_obj_t * r_in = lv_lvsfcomp_ring((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2],lv_color_from_int((int)param[3]),lv_color_from_int((int)param[4]));
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_qrcode_create:
		{
			int r_in = (int)lv_qrcode_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_qrcode_set_text:
		{
			lv_qrcode_set_text((lv_obj_t*)((int)param[0]),(char *)param[1]);
			break;
		}
		case LVFUNC_qrcode_setparam:
		{
			lv_obj_t * r_in = lv_qrcode_setparam((lv_obj_t*)((int)param[0]),(int)param[1],lv_color_from_int((int)param[2]),lv_color_from_int((int)param[3]));
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcorner_create:
		{
			int r_in = (int)lv_lvsfcorner_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_lvsfcorner_curve_text:
		{
			lv_obj_t * r_in = lv_lvsfcorner_curve_text((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcorner_zone:
		{
			lv_lvsfcorner_zone((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2],(int)param[3],(int)param[4]);
			break;
		}
		case LVFUNC_lvsfcorner_img:
		{
			lv_obj_t * r_in = lv_lvsfcorner_img((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcorner_text:
		{
			lv_obj_t * r_in = lv_lvsfcorner_text((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcorner_arc:
		{
			lv_obj_t * r_in = lv_lvsfcorner_arc((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2],lv_color_from_int((int)param[3]));
			r=JS_NewInt32(ctx,(int)r_in);
			break;
		}
		case LVFUNC_lvsfcorner_arc_scale:
		{
			lv_lvsfcorner_arc_scale((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_analogclk_create:
		{
			int r_in = (int)lv_analogclk_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_analogclk_pos_off:
		{
			lv_analogclk_pos_off((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2],(int)param[3]);
			break;
		}
		case LVFUNC_analogclk_img:
		{
			int r_in = lv_analogclk_img((lv_obj_t*)((int)param[0]),(char *)param[1],(char *)param[2],(char *)param[3],(char *)param[4]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_analogclk_refr_inteval:
		{
			lv_analogclk_refr_inteval((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_img_create:
		{
			int r_in = (int)lv_img_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_img_set_antialias:
		{
			lv_img_set_antialias((lv_obj_t*)((int)param[0]),(bool)param[1]);
			break;
		}
		case LVFUNC_img_set_offset_y:
		{
			lv_img_set_offset_y((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_img_set_offset_x:
		{
			lv_img_set_offset_x((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_img_set_zoom:
		{
			lv_img_set_zoom((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_img_set_size_mode:
		{
			lv_img_set_size_mode((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_img_set_angle:
		{
			lv_img_set_angle((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_img_get_size_mode:
		{
			int r_in = lv_img_get_size_mode((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_img_set_src:
		{
			lv_img_set_src((lv_obj_t*)((int)param[0]),(void *)param[1]);
			break;
		}
		case LVFUNC_img_get_offset_y:
		{
			int r_in = lv_img_get_offset_y((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_img_get_offset_x:
		{
			int r_in = lv_img_get_offset_x((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_img_get_zoom:
		{
			int r_in = lv_img_get_zoom((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_img_get_angle:
		{
			int r_in = lv_img_get_angle((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_img_decoder_create:
		{
			lv_img_decoder_create();
			break;
		}
		case LVFUNC_img_set_pivot:
		{
			lv_img_set_pivot((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2]);
			break;
		}
		case LVFUNC_img_get_antialias:
		{
			bool r_in = lv_img_get_antialias((lv_obj_t*)((int)param[0]));
			r=JS_NewBool(ctx,r_in);
			break;
		}
		case LVFUNC_lvsfcurve_create:
		{
			int r_in = (int)lv_lvsfcurve_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_lvsfcurve_set_buf:
		{
			lv_lvsfcurve_set_buf((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2]);
			break;
		}
		case LVFUNC_lvsfcurve_draw_arc:
		{
			lv_lvsfcurve_draw_arc((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2],(int)param[3],lv_color_from_int((int)param[4]),(int)param[5]);
			break;
		}
		case LVFUNC_lvsfcurve_set_pivot:
		{
			lv_lvsfcurve_set_pivot((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2]);
			break;
		}
		case LVFUNC_lvsfcurve_text:
		{
			lv_lvsfcurve_text((lv_obj_t*)((int)param[0]),(char *)param[1],(int)param[2],(int)param[3],lv_color_from_int((int)param[4]),(int)param[5]);
			break;
		}
		case LVFUNC_lvsfezipa_create:
		{
			int r_in = (int)lv_lvsfezipa_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_lvsfezipa_stop:
		{
			lv_lvsfezipa_stop((lv_obj_t*)((int)param[0]));
			break;
		}
		case LVFUNC_lvsfezipa_play:
		{
			lv_lvsfezipa_play((lv_obj_t*)((int)param[0]));
			break;
		}
		case LVFUNC_lvsfezipa_set_src:
		{
			lv_lvsfezipa_set_src((lv_obj_t*)((int)param[0]),(char *)param[1]);
			break;
		}
		case LVFUNC_label_create:
		{
			int r_in = (int)lv_label_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_label_set_text_static:
		{
			lv_label_set_text_static((lv_obj_t*)((int)param[0]),(char *)param[1]);
			break;
		}
		case LVFUNC_label_get_text:
		{
			char * r_in = lv_label_get_text((lv_obj_t*)((int)param[0]));
			r=JS_NewString(ctx,r_in);
			break;
		}
		case LVFUNC_label_get_text_selection_start:
		{
			int r_in = lv_label_get_text_selection_start((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_label_get_recolor:
		{
			bool r_in = lv_label_get_recolor((lv_obj_t*)((int)param[0]));
			r=JS_NewBool(ctx,r_in);
			break;
		}
		case LVFUNC_label_ins_text:
		{
			lv_label_ins_text((lv_obj_t*)((int)param[0]),(int)param[1],(char *)param[2]);
			break;
		}
		case LVFUNC_label_set_text_sel_end:
		{
			lv_label_set_text_sel_end((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_label_set_long_mode:
		{
			lv_label_set_long_mode((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_label_set_text_sel_start:
		{
			lv_label_set_text_sel_start((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_label_get_long_mode:
		{
			int r_in = lv_label_get_long_mode((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_label_set_text:
		{
			lv_label_set_text((lv_obj_t*)((int)param[0]),(char *)param[1]);
			break;
		}
		case LVFUNC_label_cut_text:
		{
			lv_label_cut_text((lv_obj_t*)((int)param[0]),(int)param[1],(int)param[2]);
			break;
		}
		case LVFUNC_label_get_text_selection_end:
		{
			int r_in = lv_label_get_text_selection_end((lv_obj_t*)((int)param[0]));
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_label_set_recolor:
		{
			lv_label_set_recolor((lv_obj_t*)((int)param[0]),(bool)param[1]);
			break;
		}
		case LVFUNC_gif_create:
		{
			int r_in = (int)lv_gif_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_gif_pause:
		{
			lv_gif_pause((lv_obj_t*)((int)param[0]));
			break;
		}
		case LVFUNC_gif_set_src:
		{
			lv_gif_set_src((lv_obj_t*)((int)param[0]),(void *)param[1]);
			break;
		}
		case LVFUNC_gif_restart:
		{
			lv_gif_restart((lv_obj_t*)((int)param[0]));
			break;
		}
		case LVFUNC_gif_resume:
		{
			lv_gif_resume((lv_obj_t*)((int)param[0]));
			break;
		}
		case LVFUNC_lvsfbarcode_create:
		{
			int r_in = (int)lv_lvsfbarcode_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_lvsfbarcode_set_text:
		{
			lv_lvsfbarcode_set_text((lv_obj_t*)((int)param[0]),(char *)param[1]);
			break;
		}
		case LVFUNC_idximg_create:
		{
			int r_in = (int)lv_idximg_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_idximg_prefix:
		{
			lv_idximg_prefix((lv_obj_t*)((int)param[0]),(char *)param[1]);
			break;
		}
		case LVFUNC_idximg_select:
		{
			lv_idximg_select((lv_obj_t*)((int)param[0]),(int)param[1]);
			break;
		}
		case LVFUNC_rlottie_create:
		{
			int r_in = (int)lv_rlottie_create((lv_obj_t*)(int)param[0]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_rlottie_raw:
		{
			int r_in = lv_rlottie_raw((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_rlottie_play:
		{
			int r_in = lv_rlottie_play((lv_obj_t*)((int)param[0]),(int)param[1]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
		case LVFUNC_rlottie_file:
		{
			int r_in = lv_rlottie_file((lv_obj_t*)((int)param[0]),(char *)param[1]);
			r=JS_NewInt32(ctx,r_in);
			break;
		}
	}
	return r;
}
