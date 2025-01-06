import * as lv from "lv";
import * as lvext from "lvext";
export class lvsfapng extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.lvsfapng_create(parent);
		this.set_obj(this.nativeobj);
	}
	stop(){
		return lvext.lvsfapng_stop(this.nativeobj);
	}
	play(){
		return lvext.lvsfapng_play(this.nativeobj);
	}
	set_src(text){
		return lvext.lvsfapng_set_src(this.nativeobj, text);
	}
}
