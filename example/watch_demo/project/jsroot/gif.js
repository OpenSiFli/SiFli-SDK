import * as lv from "lv";
import * as lvext from "lvext";
import {img} from "/img.js";
export class gif extends img {
	constructor(parent) {
		var nativeobj=lvext.gif_create(parent);
		super(parent,nativeobj);
		this.nativeobj=nativeobj;
		this.set_obj(this.nativeobj);
	}
	pause(){
		return lvext.gif_pause(this.nativeobj);
	}
	set_src(src){
		return lvext.gif_set_src(this.nativeobj, src);
	}
	restart(){
		return lvext.gif_restart(this.nativeobj);
	}
	resume(){
		return lvext.gif_resume(this.nativeobj);
	}
}
