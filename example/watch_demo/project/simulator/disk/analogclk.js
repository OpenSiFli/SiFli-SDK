import * as lv from "lv";
import * as lvext from "lvext";
export class analogclk extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.analogclk_create(parent);
		this.set_obj(this.nativeobj);
	}
	pos_off(hoff, moff, soff){
		return lvext.analogclk_pos_off(this.nativeobj, hoff, moff, soff);
	}
	img(bg, hour, min, second){
		return lvext.analogclk_img(this.nativeobj, bg, hour, min, second);
	}
	refr_inteval(start){
		return lvext.analogclk_refr_inteval(this.nativeobj, start);
	}
}
