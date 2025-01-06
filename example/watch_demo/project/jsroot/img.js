import * as lv from "lv";
import * as lvext from "lvext";
export class img extends lv.obj {
	constructor(parent, nativeobj=0) {
		super(parent,nativeobj);
		if (nativeobj!=0)
			this.nativeobj=nativeobj;
		else
			this.nativeobj=lvext.img_create(parent);
		this.set_obj(this.nativeobj);
		}
	set_antialias(glue){
		return lvext.img_set_antialias(this.nativeobj, glue);
	}
	set_offset_y(x){
		return lvext.img_set_offset_y(this.nativeobj, x);
	}
	set_offset_x(x){
		return lvext.img_set_offset_x(this.nativeobj, x);
	}
	set_zoom(start){
		return lvext.img_set_zoom(this.nativeobj, start);
	}
	set_size_mode(mode){
		return lvext.img_set_size_mode(this.nativeobj, mode);
	}
	set_angle(value){
		return lvext.img_set_angle(this.nativeobj, value);
	}
	get_size_mode(){
		return lvext.img_get_size_mode(this.nativeobj);
	}
	set_src(src){
		return lvext.img_set_src(this.nativeobj, src);
	}
	get_offset_y(){
		return lvext.img_get_offset_y(this.nativeobj);
	}
	get_offset_x(){
		return lvext.img_get_offset_x(this.nativeobj);
	}
	get_zoom(){
		return lvext.img_get_zoom(this.nativeobj);
	}
	get_angle(){
		return lvext.img_get_angle(this.nativeobj);
	}
	decoder_create(){
		return lvext.img_decoder_create(this.nativeobj);
	}
	set_pivot(x, y){
		return lvext.img_set_pivot(this.nativeobj, x, y);
	}
	get_antialias(){
		return lvext.img_get_antialias(this.nativeobj);
	}
}
