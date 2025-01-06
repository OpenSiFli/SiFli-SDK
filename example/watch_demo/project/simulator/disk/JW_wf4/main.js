import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {label} from "/label.js"
import {gif} from "/gif.js"

class wf4 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
    
    refresh() {
        var cur_time=new Date();
        if (cur_time.getMinutes()!=this.last_min) {
			this.hour.set_text(((cur_time.getHours()<=9)?"0":"")+String(cur_time.getHours()))
            this.min.set_text(((cur_time.getMinutes()<=9)?"0":"")+String(cur_time.getMinutes()));
            this.date.set_text(cur_time.toDateString());
        }
    }
    
	start() {
        // GIF image
        this.anim=new gif(this.root());
        this.anim.set_src("/JW_wf4/anim.gif");
        this.anim.set_zoom(384);
        this.anim.set_pos(80,60);

        // Dot
        this.dot=new label(this.root());
        this.dot.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -60);
        this.dot.set_text(":");
        this.dot.set_local_font(lv_enums.FONT_SUPER, lv_enums.LV_COLOR_WHITE);
        
        // Hour 
        this.hour=new label(this.root());
        this.hour.align_to(this.dot.nativeobj, lv_enums.ALIGN_OUT_LEFT_TOP, -85, 0);
        this.hour.set_local_font(lv_enums.FONT_SUPER, lv_enums.LV_COLOR_WHITE);

        // Minute
        this.min=new label(this.root());
        this.min.align_to(this.dot.nativeobj, lv_enums.ALIGN_OUT_RIGHT_TOP, 20, 0);
        this.min.set_local_font(lv_enums.FONT_SUPER, lv_enums.LV_COLOR_WHITE);

        // Date
        this.date=new label(this.root());
        this.date.align(lv_enums.ALIGN_IN_BOTTOM_MID, 0, -5);
        this.date.set_local_font(lv_enums.FONT_SUBTITLE, lv_enums.LV_COLOR_WHITE);

        this.last_min=-1;
        this.refresh();
    }
    pause() {
        this.task();
        this.anim.pause();
    }
    resume() {
		this.task(
			function() {
				this.refresh();
			}
			, 1000
		);        
        this.anim.resume();
    }    
}
globalThis.wf4 = wf4;
