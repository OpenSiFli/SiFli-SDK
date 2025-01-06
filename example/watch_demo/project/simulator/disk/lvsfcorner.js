import * as lv from "lv";
import * as lvext from "lvext";
export class lvsfcorner extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.lvsfcorner_create(parent);
		this.set_obj(this.nativeobj);
	}
	curve_text(title){
		return lvext.lvsfcorner_curve_text(this.nativeobj, title);
	}
	zone(zone, r, x, y){
		return lvext.lvsfcorner_zone(this.nativeobj, zone, r, x, y);
	}
	img(txt){
		return lvext.lvsfcorner_img(this.nativeobj, txt);
	}
	text(title){
		return lvext.lvsfcorner_text(this.nativeobj, title);
	}
	arc(start, end, color){
		return lvext.lvsfcorner_arc(this.nativeobj, start, end, color);
	}
	arc_scale(type){
		return lvext.lvsfcorner_arc_scale(this.nativeobj, type);
	}
}
