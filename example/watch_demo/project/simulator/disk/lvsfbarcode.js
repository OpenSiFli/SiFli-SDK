import * as lv from "lv";
import * as lvext from "lvext";
export class lvsfbarcode extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.lvsfbarcode_create(parent);
		this.set_obj(this.nativeobj);
	}
	set_text(text){
		return lvext.lvsfbarcode_set_text(this.nativeobj, text);
	}
}
