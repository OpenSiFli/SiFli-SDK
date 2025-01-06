import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {idximg} from "/idximg.js"
import {label} from "/label.js"

class wf2 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
    
    refresh() {
        var cur_time=new Date();
        print(cur_time.getHours());
        print(cur_time.getMinutes());
        this.time_ul.select(cur_time.getHours()/10);
        this.time_ur.select(cur_time.getHours()%10);
        this.time_bl.select(cur_time.getMinutes()/10);
        this.time_br.select(cur_time.getMinutes()%10);
        this.date_label.set_text(cur_time.toDateString());
        this.date_label.align(this.root(), lv_enums.ALIGN_IN_BOTTOM_MID, 0, -15);
    }
	start() {
        
        // Upper left
        this.time_ul=new idximg(this.root());
        this.time_ul.set_pos(27,41);
        this.time_ul.prefix("/JW_wf2/dig_1_");

        // Upper right
        this.time_ur=new idximg(this.root());
        this.time_ur.set_pos(179,51);
        this.time_ur.prefix("/JW_wf2/dig_2_");

        // Bottom left
        this.time_bl=new idximg(this.root());
        this.time_bl.set_pos(37,199);
        this.time_bl.prefix("/JW_wf2/dig_2_");

        // Bottom right
        this.time_br=new idximg(this.root());
        this.time_br.set_pos(189,209);
        this.time_br.prefix("/JW_wf2/dig_1_");
        
        this.date_label=new label(this.root());
        this.refresh();
    }
     
    resume() {
		this.task(
			function() {
				this.refresh();
			}
			, 1000
		);        
    }    
}
globalThis.wf2 = wf2;
