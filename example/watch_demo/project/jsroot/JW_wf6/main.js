import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {label} from "/label.js"
import {idximg} from "/idximg.js"
import {img} from "/img.js"

class wf6 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
    
    refresh() {
        var cur_time=new Date();
        var second=cur_time.getSeconds();
        if (second!=this.last_sec) {
            this.last_sec=second;
            this.sec_10x.select(second/10);
            this.sec_1x.select(second%10);
            var min=cur_time.getMinutes();
            if (min!=this.last_min) {
                this.last_min=min;
                this.min_10x.select(min/10);
                this.min_1x.select(min%10);
                var hour=cur_time.getHours();
                if (hour!=this.last_hour) {
                    this.last_hour=hour;
                    this.hour_10x.select(hour/10);
                    this.hour_1x.select(hour%10);
                    if (hour>11)
                        this.apm.select(1);
                    else
                        this.apm.select(0);
                    this.month.select(cur_time.getMonth()+1);
                    this.wk.select(cur_time.getDay());
                    this.day_10x.select(cur_time.getDate()/10);
                    this.day_1x.select(cur_time.getDate()%10);
                }
            }
        }
    }
    
	start() {
        // Back ground
        this.bg=new img(this.root());
        this.bg.set_src("/JW_wf6/bg.bin");
        
        // Kcal
        this.kcal=new label(this.root());
        this.kcal.align(lv_enums.ALIGN_IN_BOTTOM_MID, -110, -60);
        this.kcal.set_local_font(lv_enums.FONT_NORMAL, lv_enums.LV_COLOR_WHITE);
        this.kcal.set_text("--");
        this.kcal.bind(lv_enums.DATA_STEP,
            function(step) {
                this.set_text(String(step.calories));
            });
        this.kcal.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
        this.kcal.set_event_cb(
            function(event) {
                if (event==lv_enums.EVENT_CLICKED)
                    lv.gui_app_run("Sport");
            });

        // Heart rate
        this.hr=new label(this.root());
        this.hr.align(lv_enums.ALIGN_IN_TOP_MID, -110, 70);
        this.hr.set_local_font(lv_enums.FONT_NORMAL, lv_enums.LV_COLOR_WHITE);
        this.hr.set_text("--");
        this.hr.bind(lv_enums.DATA_HR,
            function(hr) {
                print("hr:", hr.hr);
                this.set_text(String(hr.hr));
            });
        this.hr.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
        this.hr.set_event_cb(
            function(event) {
                if (event==lv_enums.EVENT_CLICKED) {
                    print("Run HR");
                    lv.gui_app_run("Heart_rate");
                }
            });

       // Step
        this.step=new label(this.root());
        this.step.align(lv_enums.ALIGN_IN_TOP_MID, 114, 70);
        this.step.set_local_font(lv_enums.FONT_NORMAL, lv_enums.LV_COLOR_WHITE);
        this.step.set_text("--");
        this.step.bind(lv_enums.DATA_STEP,
            function(step) {
                this.set_text(String(step.step));
            });
        this.step.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
        this.step.set_event_cb(
            function(event) {
                if (event==lv_enums.EVENT_CLICKED)
                    lv.gui_app_run("Sport");
            })
            

       // Distance
        this.dist=new label(this.root());
        this.dist.align(lv_enums.ALIGN_IN_BOTTOM_MID, 114, -60);
        this.dist.set_local_font(lv_enums.FONT_NORMAL, lv_enums.LV_COLOR_WHITE);
        this.dist.set_text("--");        
        this.dist.bind(lv_enums.DATA_STEP,
            function(step) {
                var distance = step.distance * 6214 / 10000;
                this.set_text(String(step.distance));
            });
        this.dist.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
        this.dist.set_event_cb(
            function(event) {
                if (event==lv_enums.EVENT_CLICKED)
                    lv.gui_app_run("Sport");
            })

        this.dist_unit=new img(this.root());
        this.dist_unit.set_pos( 280, 387);
        this.dist_unit.set_src("/JW_wf6/dist_mi.bin");

        // Time 
        this.hour_10x=new idximg(this.root());
        this.hour_10x.set_pos(58, 183);
        this.hour_10x.prefix("/JW_wf6/num_");
        this.hour_1x=new idximg(this.root());
        this.hour_1x.set_pos(92, 183);
        this.hour_1x.prefix("/JW_wf6/num_");
        this.min_10x=new idximg(this.root());
        this.min_10x.set_pos(151, 183);
        this.min_10x.prefix("/JW_wf6/num_");
        this.min_1x=new idximg(this.root());
        this.min_1x.set_pos(185, 183);
        this.min_1x.prefix("/JW_wf6/num_");
        this.last_min=-1;
        this.sec_10x=new idximg(this.root());
        this.sec_10x.set_pos(237, 183);
        this.sec_10x.prefix("/JW_wf6/num_");
        this.sec_1x=new idximg(this.root());
        this.sec_1x.set_pos(271, 183);
        this.sec_1x.prefix("/JW_wf6/num_");
        this.dot1=new img(this.root());
        this.dot1.set_pos(120, 180);
        this.dot1.set_src("/JW_wf6/num_dot.bin")
        this.dot2=new img(this.root());
        this.dot2.set_pos(210, 180);
        this.dot2.set_src("/JW_wf6/num_dot.bin")

        // Date
        this.month=new idximg(this.root());
        this.month.set_pos(161, 267);
        this.month.prefix("/JW_wf6/mon_");
        this.wk=new idximg(this.root());
        this.wk.set_pos(74, 267);
        this.wk.prefix("/JW_wf6/week_");
        this.day_10x=new idximg(this.root());
        this.day_10x.set_pos(265, 269);
        this.day_10x.prefix("/JW_wf6/s_num_");
        this.day_1x=new idximg(this.root());
        this.day_1x.set_pos(279, 269);
        this.day_1x.prefix("/JW_wf6/s_num_");
        this.apm=new idximg(this.root());
        this.apm.set_pos(167,232);
        this.apm.prefix("/JW_wf6/apm_");
        this.last_hour=-1;

        this.last_sec=-1;
        this.refresh();
    }
    pause() {
        this.task();
    }
    resume() {
		this.task(
			function() {
				this.refresh();
			}
			, 500
		);        
    }    
}
globalThis.wf6 = wf6;
