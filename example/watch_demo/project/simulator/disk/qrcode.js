import * as lv from "lv";
import * as lvext from "lvext";
export class qrcode extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.qrcode_create(parent);
		this.set_obj(this.nativeobj);
	}
	set_text(text){
		return lvext.qrcode_set_text(this.nativeobj, text);
	}
	setparam(size, dark_color, light_color){
		return lvext.qrcode_setparam(this.nativeobj, size, dark_color, light_color);
	}
}
