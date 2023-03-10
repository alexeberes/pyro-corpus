from body_parts import *
import pickle

if __name__ == '__main__':
    body_plan = BodyCons(0, PurpleBodyPart(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.X)],
                {CubeElement.FRONT: BodyCons(1, RedBodyPart(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.X)],
                {
                CubeElement.FRONT: BodyCons(2, BlueBodyPart(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.X)],
                {
                CubeElement.RIGHT: BodyCons(3, GreenBodyPart(), [BuildSpecifications(CubeElement.RIGHT, 2, Axes.X)], None),
                CubeElement.LEFT: BodyCons(4, GreenBodyPart(), [BuildSpecifications(CubeElement.LEFT, 2, Axes.X)], None),
                CubeElement.FRONT: BodyCons(5, RedBodyPart(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.X)],
                {
                CubeElement.FRONT: BodyCons(6, BlueBodyPart(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.X)],
                {
                CubeElement.FRONT: BodyCons(7, YellowBodyPart(), [BuildSpecifications(CubeElement.FRONT, 1, Axes.X)], None),
                CubeElement.RIGHT: BodyCons(8, GreenBodyPart(), [BuildSpecifications(CubeElement.RIGHT, 2, Axes.X)], None),
                CubeElement.LEFT: BodyCons(9, GreenBodyPart(), [BuildSpecifications(CubeElement.LEFT, 2, Axes.X)], None)
                })
                })
                })
                })})
    

    with open("figure_file", "wb") as file:
        pickle.dump(Genome(0, None, body_plan), file)