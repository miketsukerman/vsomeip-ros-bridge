from email import message
from pyfranca import Processor, LexerException, ParserException, ProcessorException
from jinja2 import Template, FileSystemLoader, Environment

import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

class FIDLTraslator:
    
    def __init__(self, fidl, include):
        self.fidl = fidl 
        self.include = include 
        
        templateLoader = FileSystemLoader(searchpath=f"{dir_path}/templates/")
        templateEnv = Environment(loader=templateLoader)
        
        self.message_t = templateEnv.get_template("message.msg.j2")
        self.service_t = templateEnv.get_template("service.srv.j2")
        
    def parse(self): 

        processor = Processor()
        types = []

        try:
            print(f"importing file {self.fidl}")
            processor.import_file(self.fidl)        

            for package in processor.packages.values():
                if package.typecollections:
                    print("\tType collections:")
                    for typecollection in package.typecollections.values():       
                        if typecollection.structs:
                            print("\t\tStructs:")
                            for item in typecollection.structs.values():
                                print("\t\t- {}".format(item.name))
                                types.append(item)
                                
        except (LexerException, ParserException, ProcessorException) as e:
            print(f"ERROR: {e}")

        return types
    
    def fildTypes2RosTypes(self,name):
        types_map = {
            "Float" : "float32",
            "Double" : "float32",
            "UInt32" : "int32"
        }
        return types_map[name] if name in types_map else name
        
    def save(self, output):
        
        types = self.parse()
        
        for type in types:        
            output_file = os.path.join(output,f"{type.name}.msg")
            with open(output_file,"w+") as f:
                print(f"saving to {output_file}")
                print(type.fields)
                out = self.message_t.render(fields=type.fields, rostype=self.fildTypes2RosTypes)
                f.write(out)
