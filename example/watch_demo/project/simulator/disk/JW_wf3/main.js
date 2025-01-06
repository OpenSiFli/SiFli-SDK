import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {idximg} from "/idximg.js"
import {label} from "/label.js"
import {img} from "/img.js"

class wf3 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
    
    refresh() {
        this.anim.select(this.count);
        this.count=(this.count+1)%8;
        var cur_time=new Date();
        if (cur_time.getMinutes()!=this.last_min) {
            this.hour_10x.select(cur_time.getHours()/10);
            this.hour_1x.select(cur_time.getHours()%10);
            this.min_10x.select(cur_time.getMinutes()/10);
            this.min_1x.select(cur_time.getMinutes()%10);
            this.mon.select(cur_time.getMonth()+1);     // Month is 0-11
            this.wk.select(cur_time.getDay());      // Weekday is 0-6, Sunday is 0.
            this.date_10x.select(cur_time.getDate()/10);
            this.date_1x.select(cur_time.getDate()%10);
        }
    }
    
	start() {
        // Back ground image
        this.bg=new img(this.root());
        this.bg.set_src("/JW_wf3/bg.bin");

        // Hour 10x
        this.hour_10x=new idximg(this.root());
        this.hour_10x.set_pos(24,36);
        this.hour_10x.prefix("/JW_wf3/num61x50_");

        // Hour 1x
        this.hour_1x=new idximg(this.root());
        this.hour_1x.set_pos(87,36);
        this.hour_1x.prefix("/JW_wf3/num61x50_");

        // Min 10x
        this.min_10x=new idximg(this.root());
        this.min_10x.set_pos(165, 36);
        this.min_10x.prefix("/JW_wf3/num61x50_");

        // Min 1x
        this.min_1x=new idximg(this.root());
        this.min_1x.set_pos(228, 36);
        this.min_1x.prefix("/JW_wf3/num61x50_");

        // Comma between hour and minute
        this.time_conn=new img(this.root());
        this.time_conn.set_pos(150, 36);
        this.time_conn.set_src("/JW_wf3/num13x50_a.bin");
        
        // Month
        this.mon=new idximg(this.root());
        this.mon.set_pos(75,97);
        this.mon.prefix("/JW_wf3/mon66x13_");

        // Week
        this.wk=new idximg(this.root());
        this.wk.set_pos(24,97);
        this.wk.prefix("/JW_wf3/wee49x13_");

        // Date 10x
        this.date_10x=new idximg(this.root());
        this.date_10x.set_pos(142,97);
        this.date_10x.prefix("/JW_wf3/dat16x13_");

        // Date 1x
        this.date_1x=new idximg(this.root());
        this.date_1x.set_pos(160,97);
        this.date_1x.prefix("/JW_wf3/dat16x13_");
        
        // After Date, Chinese only
        // this.date_conn=new img(this.root());
        // this.date_conn.set_pos(178, 97);
        // this.date_conn.set_src("/JW_wf3/dat16x13_d.bin");
        
        // Animation
        this.anim=new idximg(this.root());
        this.anim.set_pos(8,131);
        this.anim.prefix("/JW_wf3/anim_");
        
        this.count=0;
        this.last_min=-1;
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
			, 160
		);        
    }    
}
globalThis.wf3 = wf3;
