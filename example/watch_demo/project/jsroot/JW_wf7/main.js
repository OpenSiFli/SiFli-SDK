import * as lv from "lv"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {idximg} from "/idximg.js"
import {label} from "/label.js"
import {img} from "/img.js"



class wf7 extends app{
	constructor() {
		super(1);		// Watch APP set 1 as parameter
	}
	refresh() {
		this.anim.select(this.count);
		this.count = (this.count+1)%16;
		var cur_time = new Date();
		if(cur_time.getMinutes() != this.last_min){
			this.hour_10x.select(cur_time.getHours()/10);
			this.hour_1x.select(cur_time.getHours()%10);
			this.min_10x.select(cur_time.getMinutes()/10);
			this.min_1x.select(cur_time.getMinutes()%10);
		}
    }
	
	start(){
		//Back ground image
		this.bg = new img(this.root());
		this.bg.set_src("/JW_wf7/cal_bg.bin");
		//this.bg.set_pos(0,0);
		
		//Hour 10x
		this.hour_10x = new idximg(this.root());
		this.hour_10x.set_pos(16,24);
		this.hour_10x.prefix("/JW_wf7/cal_num_");
		
		//Hour 1x
		this.hour_1x = new idximg(this.root());
		this.hour_1x.set_pos(81,24);
		this.hour_1x.prefix("/JW_wf7/cal_num_");
		
		//Min 10x
		this.min_10x = new idximg(this.root());
		this.min_10x.set_pos(16,129);
		this.min_10x.prefix("/JW_wf7/cal_num_");
		
		//Min 1x
		this.min_1x = new idximg(this.root());
		this.min_1x.set_pos(81,129);
		this.min_1x.prefix("/JW_wf7/cal_num_");

		//Step icon
		this.step_icon = new img(this.root());
		this.step_icon.set_src("/JW_wf7/cal_step.bin");
		this.step_icon.set_pos(104, 395);

		//step number10000x 
		this.step_num10000x = new idximg(this.root());
		this.step_num10000x.set_pos(148, 401);
		this.step_num10000x.prefix("/JW_wf7/cal_s_num_");
		this.step_num10000x.select(0);
		this.step_num10000x.bind(lv_enums.DATA_STEP,
			function(step){
					print("step_num10000x is",(step.step/10000).toFixed(2));
					this.select((step.step/10000).toFixed(2));
			})

		
		//step number1000x 
		this.step_num1000x = new idximg(this.root());
		this.step_num1000x.set_pos(170, 401);
		this.step_num1000x.prefix("/JW_wf7/cal_s_num_");
		this.step_num1000x.select(7);
		this.step_num1000x.bind(lv_enums.DATA_STEP,
			function(step){
					print("step_num1000x is",((step.step%10000)/1000).toFixed(2));
					this.select(((step.step%10000)/1000).toFixed(2));	
			})
		
		//step number100x 
		this.step_num100x = new idximg(this.root());
		this.step_num100x.set_pos(192, 401);
		this.step_num100x.prefix("/JW_wf7/cal_s_num_");
		this.step_num100x.select(0);		
		this.step_num100x.bind(lv_enums.DATA_STEP,
			function(step){
					print("step_num100x is",((step.step%1000)/100).toFixed(2));
					this.select(((step.step%1000)/100).toFixed(2));	
			})
		
		//step number10x 
		this.step_num10x = new idximg(this.root());
		this.step_num10x.set_pos(214, 401);
		this.step_num10x.prefix("/JW_wf7/cal_s_num_");
		this.step_num10x.select(0);
		this.step_num10x.bind(lv_enums.DATA_STEP,
			function(step){
					print("step_num10x is",((step.step%100)/10).toFixed(2));
					this.select(((step.step%100)/10).toFixed(2));	
			})
		
		//step number1x 
		this.step_num1x = new idximg(this.root());
		this.step_num1x.set_pos(236, 401);
		this.step_num1x.prefix("/JW_wf7/cal_s_num_");
		this.step_num1x.select(0);
		this.step_num1x.bind(lv_enums.DATA_STEP,
		function(step){
					print("step_num1x is",(step.step%10).toFixed(0));
					this.select((step.step%10).toFixed(0));	
			})

		//Animation
		this.anim = new idximg(this.root());
		this.anim.set_pos(8, 64);
		this.anim.prefix("/JW_wf7/cal_anim_");

		this.count = 0;
		this.last_min = -1;
		this.refresh();	
	}
	pause() {
		this.task();
	}
	resume(){
		this.task(
				function(){
					this.refresh();
				}
				,160
		);
	}
}

globalThis.wf7 = wf7;