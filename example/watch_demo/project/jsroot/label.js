import * as lv from "lv";
import * as lvext from "lvext";
export class label extends lv.obj {
	constructor(parent) {
		super(parent);
		this.nativeobj=lvext.label_create(parent);
		this.set_obj(this.nativeobj);
	}
	set_text_static(text){
		return lvext.label_set_text_static(this.nativeobj, text);
	}
	get_text(){
		return lvext.label_get_text(this.nativeobj);
	}
	get_text_selection_start(){
		return lvext.label_get_text_selection_start(this.nativeobj);
	}
	get_recolor(){
		return lvext.label_get_recolor(this.nativeobj);
	}
	ins_text(pos, txt){
		return lvext.label_ins_text(this.nativeobj, pos, txt);
	}
	set_text_sel_end(index){
		return lvext.label_set_text_sel_end(this.nativeobj, index);
	}
	set_long_mode(long_mode){
		return lvext.label_set_long_mode(this.nativeobj, long_mode);
	}
	set_text_sel_start(index){
		return lvext.label_set_text_sel_start(this.nativeobj, index);
	}
	get_long_mode(){
		return lvext.label_get_long_mode(this.nativeobj);
	}
	set_text(text){
		return lvext.label_set_text(this.nativeobj, text);
	}
	cut_text(pos, cnt){
		return lvext.label_cut_text(this.nativeobj, pos, cnt);
	}
	get_text_selection_end(){
		return lvext.label_get_text_selection_end(this.nativeobj);
	}
	set_recolor(glue){
		return lvext.label_set_recolor(this.nativeobj, glue);
	}
}
