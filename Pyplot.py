import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def afficher_vecteurs(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # origine (optionnel)
    origin = Point(0, 0, 0)

    for p in points:
        # tracer le point
        ax.scatter(p.x, p.y, p.z)

        # tracer vecteur depuis origine
        ax.quiver(
            origin.x, origin.y, origin.z,
            p.x, p.y, p.z,
            arrow_length_ratio=0.1
        )

    # labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()


def main():
    p1 = Point(1, 2, 3)
    p2 = Point(4, 1, 2)
    p3 = Point(2, 5, 1)

    points = [p1, p2, p3]
    afficher_vecteurs(points)


if __name__ == "__main__":
    main()


def afficher_vecteurs_entre_points(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i in range(len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]

        ax.scatter(p1.x, p1.y, p1.z)
        ax.scatter(p2.x, p2.y, p2.z)

        ax.quiver(
            p1.x, p1.y, p1.z,
            p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z,
            arrow_length_ratio=0.1
        )

    plt.show()