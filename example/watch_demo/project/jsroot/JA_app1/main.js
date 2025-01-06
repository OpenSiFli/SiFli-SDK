import * as lv from "lv"
import * as os from "os"
import {app} from "lvapp"
import * as lv_enums from "/lv_enums.js"
import {qrcode} from "/qrcode.js"
import {lvsfbarcode} from "/lvsfbarcode.js"

function arrayBufferToString(buffer){
    var arr = new Uint8Array(buffer);
    var str = String.fromCharCode.apply(String, arr);
    return str;
}

class app1 extends app{	
	start() {
		
		// Demo for QRcode		
        this.qrcode = new qrcode(this.root());
        this.qrcode.setparam((448/3),lv_enums.LV_COLOR_WHITE, lv_enums.LV_COLOR_BLACK);
		this.qrcode.set_text("0123456-ABC-abcd",2); 
		print("ALIGN_IN_TOP_MID=",lv_enums.ALIGN_IN_TOP_MID)
        this.qrcode.align(lv_enums.ALIGN_IN_TOP_MID, 0,(448/6));
        this.qrcode.set_event_cb(
			function(event){
				print("qrcode event ", event);
                print(lv_enums.EVENT_CLICKED);
				if (event==lv_enums.EVENT_CLICKED) {
					lv.gui_app_self_exit();
                }
			}
		);
		print("Qrcode created");
		// Demo for Barcode
        this.barcode = new lvsfbarcode(this.root());
		this.barcode.set_text("0705632441974");
        this.barcode.align(lv_enums.ALIGN_IN_TOP_MID, 0, (448*2/3));
        this.barcode.set_event_cb(
			function(event){
				print("barcode event ", event);
			}
		);  
		print("Barcode created");
		
		// Demo for task
		this.count=0;
		this.task(
			function() {
				print("Task called ", this.count);
				this.count++;
			}
			, 1000
		);
        print("Started\n");
    }    
	resume() {
		// Demo for OS file access
		const buffer = new ArrayBuffer(8);
		this.f=os.open("/gif.js", os.O_RDONLY);
		os.read(this.f,buffer,0,5);
		print("Reume:", arrayBufferToString(buffer));
		os.close(this.f);
		
		// Demo for OS directory
		os.chdir("JA_app1");
		print(os.getcwd());
		var local_files=os.readdir("/");
		print(local_files[0].length);
		for (var i=0;i<local_files[0].length;i++)
		{ 		
			print(local_files[0][i]);
		}		
	}
}
globalThis.app1 = app1;
