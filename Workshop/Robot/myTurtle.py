import ipyturtle3 as ipyturtle

class Turtle(ipyturtle.Turtle):

    def __init__(self,width=500,height=500):
        myCanvas=ipyturtle.Canvas(width=width,height=height)
        display(myCanvas)
        myTS=ipyturtle.TurtleScreen(myCanvas)
        myTS.clear()
        myTS.bgcolor("lightgray")

        super().__init__(myTS)
        #jess=turtle.Turtle(myTS,isHolonomic=True)
        #jess.shape("square")
        self.shape("turtle")
