import * as lv from "lv";
import * as lvext from "lvext";
import {img} from "/img.js";
export class idximg extends img {
	constructor(parent) {
		var nativeobj=lvext.idximg_create(parent);
		super(parent,nativeobj);
		this.nativeobj=nativeobj;
		this.set_obj(this.nativeobj);
	}
	prefix(text){
		return lvext.idximg_prefix(this.nativeobj, text);
	}
	select(start){
		return lvext.idximg_select(this.nativeobj, start);
	}
}
