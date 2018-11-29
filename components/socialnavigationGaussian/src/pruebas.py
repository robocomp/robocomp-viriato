#
# removePoints
#
def removePoints(self, listp):
    print("Remove Points")
    aux = listp
    polylines = []
    print("--------------------------------------------------------")
    print("Hay ", len(listp), "polilineas")
    #######################################################################
    change = False

    for i in xrange(len(listp)):
        print("creo poligono para polilinea ", listp.index(listp[i]) + 1)
        qp = QPolygonF()
        for p in listp[i]:
            qp.append(QPointF(p.x, p.z))
            # plt.axis('equal')
            # plt.plot(p.x, p.z, "*g-")

        polycont = []
        for j in xrange(len(listp)):
            if i != j:
                for point in listp[j]:
                    if qp.containsPoint(QPointF(point.x, point.z), Qt.OddEvenFill):
                        polycont.append(listp[j])
                        print("Polilinea ", listp.index(listp[i]) + 1, "contiene a polilinea ",
                              listp.index(listp[j]) + 1)
                        change = True
                        break

        if change:

            for p in polycont:
                if p in listp:
                    listp[i].extend(p)
                    listp.pop(listp.index(p))
                else:

                    print("La polilinea no esta en aux")
                    # poly.extend(p)

    # UUUUH BIG PROBLEM

    if change:
        print("len de aux", len(aux))
        for pl in listp[:]:
            points = []
            for pn in pl:
                points.append([pn.x, pn.z])

            points = np.asarray(points)
            hull = ConvexHull(points)
            listp.remove(pl)
            listp.append(points[hull.vertices])

        for pol in listp:
            polyline = []
            for pnt in pol:
                punto = SNGPoint2D()
                punto.x = pnt[0]
                punto.z = pnt[1]
                polyline.append(punto)
            polylines.append(polyline)


    else:
        polylines = aux
    print("--------------------------------------------------------")
    # plt.figure()
    # for ps in polylines:
    #     for p in ps:
    #         plt.plot(p.x, p.z, "*r-")
    #         plt.axis('equal')
    #         plt.xlabel('X')
    #         plt.ylabel('Y')
    # plt.show()

    return polylines