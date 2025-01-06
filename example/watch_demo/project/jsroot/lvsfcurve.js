import * as lv from "lv";
import * as lvext from "lvext";
export class lvsfcurve extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.lvsfcurve_create(parent);
		this.set_obj(this.nativeobj);
	}
	set_buf(start, end){
		return lvext.lvsfcurve_set_buf(this.nativeobj, start, end);
	}
	draw_arc(r, start_angle, end_angle, color, width){
		return lvext.lvsfcurve_draw_arc(this.nativeobj, r, start_angle, end_angle, color, width);
	}
	set_pivot(x, y){
		return lvext.lvsfcurve_set_pivot(this.nativeobj, x, y);
	}
	text(text, angle, r, color, size){
		return lvext.lvsfcurve_text(this.nativeobj, text, angle, r, color, size);
	}
}
