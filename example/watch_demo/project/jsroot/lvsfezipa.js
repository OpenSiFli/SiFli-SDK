import * as lv from "lv";
import * as lvext from "lvext";
export class lvsfezipa extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.lvsfezipa_create(parent);
		this.set_obj(this.nativeobj);
	}
	stop(){
		return lvext.lvsfezipa_stop(this.nativeobj);
	}
	play(){
		return lvext.lvsfezipa_play(this.nativeobj);
	}
	set_src(text){
		return lvext.lvsfezipa_set_src(this.nativeobj, text);
	}
}
