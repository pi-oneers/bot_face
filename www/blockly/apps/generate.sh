#!/bin/sh
java -jar _soy/SoyMsgExtractor.jar --outputFile extracted_msgs.xlf --srcs common.soy,code/template.soy,graph/template.soy,maze/template.soy,plane/template.soy,puzzle/template.soy,turtle/template.soy,index/template.soy;
../i18n/xliff_to_json.py --xlf extracted_msgs.xlf --templates common.soy {code,graph,index,maze,plane,puzzle,turtle}/template.soy;
../i18n/json_to_js.py --output_dir=code/generated --template common.soy,code/template.soy json/*.json;
