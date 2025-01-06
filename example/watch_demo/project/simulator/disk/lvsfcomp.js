import * as lv from "lv";
import * as lvext from "lvext";
export class lvsfcomp extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.lvsfcomp_create(parent);
		this.set_obj(this.nativeobj);
	}
	img(txt){
		return lvext.lvsfcomp_img(this.nativeobj, txt);
	}
	set_ring(index, scale){
		return lvext.lvsfcomp_set_ring(this.nativeobj, index, scale);
	}
	set_type(type){
		return lvext.lvsfcomp_set_type(this.nativeobj, type);
	}
	arc(scale, color, bg_color){
		return lvext.lvsfcomp_arc(this.nativeobj, scale, color, bg_color);
	}
	set_arc(type){
		return lvext.lvsfcomp_set_arc(this.nativeobj, type);
	}
	text(txt){
		return lvext.lvsfcomp_text(this.nativeobj, txt);
	}
	ring(index, scale, color, bg_color){
		return lvext.lvsfcomp_ring(this.nativeobj, index, scale, color, bg_color);
	}
}
