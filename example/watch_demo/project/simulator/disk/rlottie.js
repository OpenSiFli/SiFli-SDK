import * as lv from "lv";
import * as lvext from "lvext";
export class rlottie extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.rlottie_create(parent);
		this.set_obj(this.nativeobj);
	}
	raw(path){
		return lvext.rlottie_raw(this.nativeobj, path);
	}
	play(enable){
		return lvext.rlottie_play(this.nativeobj, enable);
	}
	file(path){
		return lvext.rlottie_file(this.nativeobj, path);
	}
}
