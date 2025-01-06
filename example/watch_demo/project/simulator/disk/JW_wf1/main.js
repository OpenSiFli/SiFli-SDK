import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {analogclk} from "/analogclk.js"

class wf1 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
	start() {
        this.anaclk=new analogclk(this.root());
        this.anaclk.pos_off(10,10,85);
        this.anaclk.img("/JW_wf1/bg.bin","/JW_wf1/hour.bin","/JW_wf1/minute.bin","/JW_wf1/second.bin");
        this.rate=30;
	}	
    pause() {
        this.anaclk.refr_inteval(0);
    }    
    resume() {
        this.anaclk.refr_inteval(this.rate);
    }    
}
globalThis.wf1 = wf1;
