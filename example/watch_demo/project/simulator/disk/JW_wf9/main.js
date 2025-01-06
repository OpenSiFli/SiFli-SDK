import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {img} from "/img.js"
import {lvsfcomp} from "/lvsfcomp.js"
import {lvsfcorner} from "/lvsfcorner.js"
import {label} from "/label.js"
import {analogclk} from "/analogclk.js"

var LV_HOR_RES_MAX = lv.get_hor_max();
var LV_VER_RES_MAX = lv.get_ver_max();
var MODULAR_SMALL_SIZE = (LV_HOR_RES_MAX / 5);
var TEXT_ANGLE = 4;
var GAP_ANGLE_BETWEEN_TXT_ARC = (TEXT_ANGLE>>1);
var CLOCK_MODULAR_PADDING =  (MODULAR_SMALL_SIZE * 5 /4) - 10;

class wf9 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
	
	refresh(){
		var cur_time=new Date();;
		var secs=cur_time.getSeconds();
		var draw_bl_corner=true;
		var mil_secs=cur_time.getMilliseconds();

		if(this.last_s != secs){
			this.last_s = secs;
			if(this.last_day != cur_time.getDate())
			{
				this.last_day = cur_time.getDate();
				this.mod_date.set_text(String(this.last_day));
			}
		}

		if(draw_bl_corner){
			this.redraw_quad(2,hours,mins,secs,mil_secs);
			this.last_ms=mil_secs;
		}

	}
	
	redraw_quad(index,h,m,s,ms){
		var corner = this.corner_container[index];
		switch(index){
		case 0:
			corner.arc_scale(s*100/60);	
			var buff = "#FFFFFF "+String(h).padStart(2,'0')+"#";
			corner.text(buff);
		break;
		case 1:
			var buff = "  #FD9426 "+String(m).padStart(2,'0')+":"+String(s).padStart(2,'0')+","+String(h).padStart(2,'0')+"HRS#";	
			corner.curve_text(buff);
			corner.text("CUP");
		break;
		case 2:
			//print("redraw_quad , set time");
			var buff = "   #FD9426 "+String(m).padStart(2,'0')+":"+String(s).padStart(2,'0')+"."+String(ms).padStart(3,'0')+"#";
			corner.curve_text(buff);							
		break;
		case 3:
			var buff = "  #FD9426 "+String(m).padStart(2,'0')+":"+String(s).padStart(2,'0')+"#";
			corner.curve_text(buff);
			corner.arc_scale(s*100/60);
		break;			
		}
	}
	
	graphic_corner(layer){
		var corner_r = (LV_HOR_RES_MAX > LV_VER_RES_MAX) ? (LV_VER_RES_MAX>>1) : (LV_HOR_RES_MAX>>1);
        print(corner_r);
        print(LV_VER_RES_MAX);
        print(LV_HOR_RES_MAX);
		this.corner_container = new Array();
		for (var i = 0; i < 4; i++) 
		{
			this.corner_container[i] = new lvsfcorner(layer);
			this.corner_container[i].add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
			this.corner_container[i].zone(i+1, corner_r - corner_r/40,LV_HOR_RES_MAX>>1, LV_VER_RES_MAX>>1);
			switch(i+1)
			{
				case 1: //top_right battery
				{
					this.corner_container[i].curve_text("  #FD9426 0       100#");
					var arc_angles = (TEXT_ANGLE * 8)>>1;
					var end = 315 + (arc_angles - GAP_ANGLE_BETWEEN_TXT_ARC) - 5;
					var start = 315 - (arc_angles - GAP_ANGLE_BETWEEN_TXT_ARC) ;
					this.corner_container[i].arc(start, end, lv.color_make(253, 148, 38));
					var buff = "#FFFFFF 100 #";
					this.corner_container[i].text(buff);
					this.corner_container[i].set_event_cb(
						function(event) {
							if (event==lv_enums.EVENT_CLICKED)
								lv.jump_to_setting_subpage("Setting","electric", lv_enums.ELECTRIC);
					});

					this.corner_container[i].bind(lv_enums.DATA_BATTERY,function(battery){
						print("battery : ", battery.level);
						this.arc_scale(battery.level);
						var buff = "#FFFFFF "+String(battery.level)+"#";
						this.text(buff);
					});
					break;
				}
				case 2: //top left alarm
					this.corner_container[i].img("/JW_wf9/gra_alarm.bin");
					this.corner_container[i].curve_text("  #FD9426 alarm 10:27#");
					this.corner_container[i].set_event_cb(
						function(event) {
							if (event==lv_enums.EVENT_CLICKED)
								lv.gui_app_run("Alarm");
					});
				break;
				case 3: //bottom left dynamic time
					this.corner_container[i].img("/JW_wf9/gra_clock2.bin");
				break;
				case 4://bottom right countdown
				{
					this.corner_container[i].img("/JW_wf9/gra_clock1.bin");
					var str_angles = TEXT_ANGLE * 13;
					var start = 45 - (str_angles >> 1);
					var end = (start + (TEXT_ANGLE * 7) - TEXT_ANGLE);
					this.corner_container[i].arc(start, end, lv.color_make(253, 148, 38));
					this.corner_container[i].set_event_cb(
						function(event) {
							if (event==lv_enums.EVENT_CLICKED)
								lv.gui_app_run("Countdown");
					});
					break;
				}
			}
			//this.redraw_quad(i,0,0,0,0);
		}	
	}

	modular_small(par){
		this.mod_sport = new img(par);
		this.mod_sport.set_src("/JW_wf9/comp_run.bin");
		this.mod_sport.align(lv_enums.ALIGN_CENTER, 0, -CLOCK_MODULAR_PADDING);
        this.mod_sport.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
        this.mod_sport.set_event_cb(
            function(event) {
                if (event==lv_enums.EVENT_CLICKED)
                    lv.gui_app_run("Sport");
        });

		this.mod_music = new img(par);
		this.mod_music.set_src("/JW_wf9/comp_music.bin");
		this.mod_music.align(lv_enums.ALIGN_CENTER, -CLOCK_MODULAR_PADDING, 0);
        this.mod_music.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
        this.mod_music.set_event_cb(
            function(event) {
                if (event==lv_enums.EVENT_CLICKED)
                    lv.gui_app_run("Music");
        });

		this.mod_activity = new lvsfcomp(par);
		this.mod_activity.set_type(lv_enums.COMP_MOD_SMALL);
		this.mod_activity.align(lv_enums.ALIGN_CENTER, CLOCK_MODULAR_PADDING, 0);
        this.mod_activity.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
		for (var i = 0; i < 3; i++)
		{
			switch (i)
			{
			case 0:
				this.mod_activity.ring(i, 0, lv.color_make(0xff,0x1b, 0x37), lv.color_make(0x33, 0x05, 0x11));
                this.mod_activity.set_ring(i, 50);
				break;
			case 1:
				this.mod_activity.ring(i, 0, lv.color_make(0x3f, 0xff, 14), lv.color_make(0x1c, 0x33, 0x00));
                this.mod_activity.set_ring(i, 30);
				break;
			default:
				this.mod_activity.ring(i, 0, lv.color_make(0x00, 0xea, 0xff), lv.color_make(0x00, 0x2f, 0x33));
                this.mod_activity.set_ring(i, 101);
				break;
			}
		}
		this.mod_activity.set_event_cb(function(event){
			if(event==lv_enums.EVENT_CLICKED)
				lv.gui_app_run("Activity");
		});
		this.mod_activity.bind(lv_enums.DATA_STEP,function(step){
			this.set_ring(0, step.step * 100 / 10000);
			this.set_ring(1, step.distance * 100 / 10000);
			this.set_ring(2, step.calories * 100 / 10000);
		});

		this.mod_date = new label(par);
        this.mod_date.set_local_font(lv_enums.FONT_BIGL, lv_enums.LV_COLOR_WHITE);
        this.mod_date.set_text("--");        
        this.mod_date.add_flag(lv_enums.LV_OBJ_FLAG_CLICKABLE);
		this.mod_date.set_event_cb(function(event){
			if(event==lv_enums.EVENT_CLICKED)
				lv.gui_app_run("Calendar");
		})
		this.mod_date.align(lv_enums.ALIGN_CENTER,0, CLOCK_MODULAR_PADDING);
	}

	start(){

		this.graphic_corner(this.root());

		this.bg = new img(this.root());
		this.bg.set_src("/JW_wf9/clock_bg.bin");
		this.bg.align(lv_enums.ALIGN_CENTER, 0, 0);

		this.modular_small(this.root());

		this.clk = new analogclk(this.root());
        this.clk.pos_off(8,8,26);
        this.clk.img(0,"/JW_wf9/clock_hour.bin","/JW_wf9/clock_min.bin","/JW_wf9/clock_sec.bin");

		this.last_s=-1;
		this.update_obj=7;
		this.last_ms=-1;
		this.last_day = -1;
	}
	
	pause() {
		this.task();
        this.clk.refr_inteval(0);
    } 
	
    resume() {
		this.task(
			function(){
				this.refresh();
			}
			, 1000
		);
        this.clk.refr_inteval(30);
    }
}

globalThis.wf9 = wf9;
