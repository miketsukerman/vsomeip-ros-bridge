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
                                types.append(item.name)
                                
        except (LexerException, ParserException, ProcessorException) as e:
            print(f"ERROR: {e}")

        return types
    
        
    def save(self, output):
        
        types = self.parse()

        for type in types:        
            output_file = f"{output}/{type}.msg"
            with open(output_file,"w+") as f:
                print(f"saving to {output_file}")
                out = self.message_t.render()
                f.write(out)
            
