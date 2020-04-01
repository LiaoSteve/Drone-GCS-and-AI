class Human():
    def __init__(self,h,w):        
        self.height=h
        self.weight=w    
    def test(self):
        return self.height   

class Human2():
    def __init__(self,h2,w2):      
        self.height2=h2
        self.weight2=w2
    def test2(self):
        return self.height2    

class Woman(Human,Human2):
    def __init__(self,h,w,h2,w2,bust=0,waist=0,hip=0):
        Human.__init__(self,h,w)
        Human2.__init__(self,h2,w2)           
    def test3(self):
        return self.height
    

a = Woman(165,54,170,56,83,64,84)
print(a.test2())