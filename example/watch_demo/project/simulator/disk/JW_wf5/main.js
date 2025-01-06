import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {label} from "/label.js"
import {gif} from "/gif.js"
import {idximg} from "/idximg.js"

class wf5 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
    
    refresh() {
        var cur_time=new Date();
        if (cur_time.getMinutes()!=this.last_min) {
            this.tm.set_text(String(cur_time.getHours())+'-'+String(cur_time.getMinutes()));
            this.date.set_text(cur_time.getFullYear()+'-'+ (cur_time.getMonth()+1)+'-'+ cur_time.getDate());
        }
    }
    
	start() {
        // GIF image
        this.anim=new gif(this.root());
        this.anim.set_src("/JW_wf5/anim.gif");
        this.anim.set_zoom(455);
        this.anim.align(lv_enums.ALIGN_CENTER, 0, 0);

        // Time
        this.tm=new label(this.root());
        this.tm.align(lv_enums.ALIGN_IN_TOP_LEFT, 40, 10);
        this.tm.set_local_font(lv_enums.FONT_BIGL, lv_enums.LV_COLOR_WHITE);

        // Date
        this.date=new label(this.root());
        this.date.align_to(this.tm.nativeobj, lv_enums.ALIGN_OUT_BOTTOM_MID, 0, 40);
        this.date.set_local_font(lv_enums.FONT_TITLE, lv_enums.LV_COLOR_WHITE);

        // Weather
        this.weather=new idximg(this.root());
        this.weather.align(lv_enums.ALIGN_IN_TOP_RIGHT, -20,20);
        this.weather.prefix("/JW_wf5/wt_");
        this.weather.select(0);
        this.weather.bind(lv_enums.DATA_WEATHER_INFO,
            function(weather) {
                print("Weather state:", weather.state);
                this.select(weather.state);
            }
            );

        // temperature
        this.temp=new label(this.root());
        this.temp.align_to(this.weather.nativeobj, lv_enums.ALIGN_OUT_LEFT_MID, -30, -15);
        this.temp.set_local_font(lv_enums.FONT_TITLE, lv_enums.LV_COLOR_WHITE);
        this.temp.set_text("--°C");
        this.temp.bind(lv_enums.DATA_WEATHER_INFO,
            function(weather) {
                print("Weather temperature:", weather.temperature);
                this.set_text(weather.temperature+"°C");
            }
        )
        
        this.last_min=-1;
        this.refresh();
    }
    pause() {
        this.task();
        this.anim.pause(0);
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
globalThis.wf5 = wf5;
