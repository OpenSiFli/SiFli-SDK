import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {lvsfcomp} from "/lvsfcomp.js"
import {lvsfcorner} from "/lvsfcorner.js"
import {img} from "/img.js"
import {analogclk} from "/analogclk.js"
import {lvsfcurve} from "/lvsfcurve.js"
import {rlottie} from "/rlottie.js"

var LV_HOR_RES_MAX = lv.get_hor_max();
var LV_VER_RES_MAX = lv.get_ver_max();

var CIRCULAR_SIZE = (LV_HOR_RES_MAX / 6);

var MODULAR_SMALL_SIZE = (LV_HOR_RES_MAX / 5);

var EXTRA_LARGE_SIZE = (LV_HOR_RES_MAX / 2);

var TEXT_ANGLE = 4;

var GAP_ANGLE_BETWEEN_TXT_ARC = (TEXT_ANGLE>>1);
var CLOCK_MODULAR_PADDING =  (MODULAR_SMALL_SIZE * 5 /4)-10;

var BG_ROOM_FACTOR = 256;

var HAND_HOUR_PIVOT_TO_BOTTOM  =  15;
var HAND_MINUTE_PIVOT_TO_BOTTOM  =   12;
var HAND_SECOND_PIVOT_TO_BOTTOM   =  63;

class wf8 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
	
	refresh(){
		var cur_time=new Date();
		var hours=cur_time.getHours();
		var mins=cur_time.getMinutes();
		var secs=cur_time.getSeconds();
		var draw_bl_corner=true;
		var mil_secs=cur_time.getMilliseconds();
		if(this.last_s != secs){
			switch(this.update_obj){
				case 1:
					this.redraw_quad(1,hours,mins,secs,mil_secs);
					this.update_obj++;
					break;
				case 2:
					this.redraw_quad(3,hours,mins,secs,mil_secs);
					this.update_obj++;
					break;
				case 3:
					this.small_redraw(0,hours,mins,secs,mil_secs);
					this.update_obj++;				
					break;
				case 4:
					this.small_redraw(1,hours,mins,secs,mil_secs);
					this.update_obj++;				
					break;
				case 5:
					this.small_redraw(2,hours,mins,secs,mil_secs);
					this.update_obj=7;				
					break;
				default:
					this.redraw_quad(0,hours,mins,secs,mil_secs);
					this.update_obj=1;				
					break;
			}
			this.last_s=secs;
		}
		if(-1!=this.last_ms){
			if((mil_secs>=this.last_ms)&&(mil_secs-this.last_ms)<100){
				draw_bl_corner=false;
			}else if((mil_secs<this.last_ms)&&(mil_secs+1000-this.last_ms)<100){
				draw_bl_corner=false;	
			}
		}
		if(draw_bl_corner){
			this.redraw_quad(2,hours,mins,secs,mil_secs);
			this.last_ms=mil_secs;
		}
	}
	
	get_time(sec_need){
		var cur_time=new Date();
		var hour=cur_time.getHours();
		var min=cur_time.getMinutes();
		var sec="";
		if(sec_need){
			sec+=":"+String(cur_time.getSeconds()).padStart(2,'0');
		}
		
		print(String(hour)+':'+String(min).padStart(2,'0')+sec);
		return String(hour)+':'+String(min).padStart(2,'0')+sec;
	}
	
	draw_sifli(par){
		var curve_txt_str="S I F L I";
		var txt_r=(LV_HOR_RES_MAX>>1) * BG_ROOM_FACTOR / 256;
		var text_angles = (TEXT_ANGLE) * (curve_txt_str.length);
		var start_angle = 180- (text_angles>>1);
		var pivot_x =LV_HOR_RES_MAX>>1;
		var pivot_y = LV_VER_RES_MAX>>1;
		
		var txt_width = (lv.trigo_sin((text_angles>>1)) * 2 * txt_r)>>15;
		var txt_height = txt_r - ((lv.trigo_sin(90 - (text_angles>>1)) * (txt_r - lv.lvsf_font_height(lv_enums.FONT_NORMAL)))>>15);
		txt_width += (lv.lvsf_font_width(1,'\0')>>1);
		
		this.curve = new lvsfcurve(par);
		this.curve.set_buf(txt_width,txt_height);
		print("curve_text x ", pivot_x, pivot_y, txt_width, txt_height);
		print("txt_angle   ", text_angles, txt_r);
		this.curve.set_pivot(pivot_x, pivot_y);
		this.curve.draw_arc(txt_r, 180-(text_angles>>1),180+(text_angles>>1),lv.color_make(0x80,0x80,0x80),20);
		this.curve.text(curve_txt_str, 88+((text_angles+TEXT_ANGLE)>>1),
			txt_r-(lv.lvsf_font_height(lv_enums.FONT_NORMAL)>>2),lv.color_make(0x00,0x80,0x80),lv_enums.FONT_NORMAL);
		this.curve.align(lv_enums.ALIGN_IN_TOP_MID,0,((LV_VER_RES_MAX-LV_HOR_RES_MAX)>>1)+10);
	}
	
	small_redraw(id,h,m,s,ms){
		switch(id){
			case 0:
				this.mod_music.set_ring(0, s * 16 / 10);
			break;
			case 1:
				var cur_time = new Date();
				var month=cur_time.getMonth()+1;
				var day=cur_time.getDate();
				var buff="\n   #FF9600 "+String(month)+" - "+String(day)+"\n #FFFFFF  "+this.get_time(true);
				this.mod_calendar.text(buff);
			break;
			case 2:
				this.mod_sport.set_ring(0, s * 100 / 60);
				this.mod_sport.set_ring(1, m * 100 / 60);
				this.mod_sport.set_ring(2, h * 100 / 60);
			break;			
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
		var corner_r = ((LV_HOR_RES_MAX > LV_VER_RES_MAX) ? LV_VER_RES_MAX : LV_HOR_RES_MAX)>>1;
		this.corner_container = new Array();
		for (var i = 0; i < 4; i++) 
		{
			var a = new lvsfcorner(layer);
			a.zone(i+1, corner_r - corner_r/40,LV_HOR_RES_MAX>>1, LV_VER_RES_MAX>>1);
			switch(i+1)
			{
				case 1:
				{
					a.curve_text("  #FD9426 55      76#");
					var arc_angles = (TEXT_ANGLE * 6)>>1;
					var end = 315 + (arc_angles - GAP_ANGLE_BETWEEN_TXT_ARC);
					var start = 315 - (arc_angles - GAP_ANGLE_BETWEEN_TXT_ARC);
					a.arc(start, end, lv.color_make(253, 148, 38));
					break;
				}
				case 2:
					a.text("CUP");
				break;
				case 3:
					a.img("/JW_wf8/gra_clock2.bin");
				break;
				case 4:
				{
					a.img("/JW_wf8/gra_clock1.bin");
					var str_angles = TEXT_ANGLE * 13;
					var start = 45 - (str_angles >> 1);
					var end = (start + (TEXT_ANGLE * 7) - TEXT_ANGLE);
					a.arc(start, end, lv.color_make(253, 148, 38));
					break;
				}
			}
			this.corner_container[i] = a;
			this.redraw_quad(i,0,0,0,0);
		}	
	}

	modular_small(par){
		var cur_time=new Date();
		var second=cur_time.getSeconds();
		var month=cur_time.getMonth()+1;
		var day=cur_time.getDate();
		
		this.mod_calendar = new lvsfcomp(par);
		this.mod_calendar.set_type(lv_enums.COMP_MOD_SMALL);
		var buff="\n   #FF9600 "+String(month)+' - '+String(day)+'\n #FFFFFF  '+this.get_time(true);
		this.mod_calendar.text(buff);
		this.mod_calendar.align(lv_enums.ALIGN_CENTER, 0, - CLOCK_MODULAR_PADDING);	
	
		this.mod_music = new lvsfcomp(par);
		this.mod_music.set_type(lv_enums.COMP_MOD_SMALL);
		this.mod_music.ring(0, 63, lv.color_make(106, 198, 222), lv.color_make(13, 34, 40));
		this.mod_music.img("/JW_wf8/ms_water.bin");
		this.mod_music.align(lv_enums.ALIGN_CENTER, -CLOCK_MODULAR_PADDING, 0);
		
		this.mod_sport = new lvsfcomp(par);
		this.mod_sport.set_type(lv_enums.COMP_MOD_SMALL);
	
		for (var i = 0; i < 3; i++)
		{
			switch (i)
			{
			case 0:
				this.mod_sport.ring(i, 0, lv.color_make(200, 100, 50), lv.color_make(40, 34, 10));
				break;
			case 1:
				this.mod_sport.ring(i, 0, lv.color_make(50, 198, 20), lv.color_make(13, 34, 5));
				break;
			default:
				this.mod_sport.ring(i, 0, lv.color_make(106, 198, 222), lv.color_make(13, 34, 40));
				break;
			}
		}
		this.mod_sport.align(lv_enums.ALIGN_CENTER, CLOCK_MODULAR_PADDING, 0);
		
		this.lottie = new rlottie(par);
        this.lottie.set_size(LV_HOR_RES_MAX/5, LV_HOR_RES_MAX/5);
		this.lottie.file("/JW_wf8/happy.json");
		this.lottie.align(lv_enums.ALIGN_CENTER,0, CLOCK_MODULAR_PADDING);
	}

	start(){

		this.bg = new img(this.root());
		this.bg.set_src("/JW_wf8/clock_dashes.bin");
		this.bg.align(lv_enums.ALIGN_CENTER, 0, 0);

		this.draw_sifli(this.root());
		
		//create corner obj
		this.graphic_corner(this.root());
		
		this.modular_small(this.root());

        this.anaclk=new analogclk(this.root());
        this.anaclk.pos_off(HAND_HOUR_PIVOT_TO_BOTTOM,HAND_MINUTE_PIVOT_TO_BOTTOM,HAND_SECOND_PIVOT_TO_BOTTOM);
        this.anaclk.img(0,"/JW_wf8/hour_hand.bin","/JW_wf8/minute_hand.bin","/JW_wf8/second_hand.bin");
        this.rate=30;

		this.last_s=-1;
		this.update_obj=7;
		this.last_ms=-1;
		this.redraw_interv_ms=30;
		this.step_ms=30;
		this.rate = 30;
	}
	
	pause() {
		this.task();
		this.anaclk.refr_inteval(0);
        this.lottie.play(0);
    } 
	
    resume() {
		this.anaclk.refr_inteval(this.rate);
        this.lottie.play(1);
		this.task(
			function(){
				this.refresh();
			}
			, 30
		);
    }
}

globalThis.wf8 = wf8;
