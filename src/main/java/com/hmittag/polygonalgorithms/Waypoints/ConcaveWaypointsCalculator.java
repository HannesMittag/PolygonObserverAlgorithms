package com.hmittag.polygonalgorithms.Waypoints;

import com.hmittag.polygonalgorithms.JTS.JtsHelper;
import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ConcaveWaypointsCalculator {
    //region fields
    private ConvexWaypointsCalculator convexWaypointsCalculator;

    //test
    private GraphicsContext graphicsContext;
    private int test = 0;
    //endregion

    //region compute
    public List<Vector> computeMission(Polygon polygon, double distanceBetweenLines, double distanceToBoundaries, double linesOffset, double angle, ConvexWaypointsCalculator.StartingPosition startingPosition) {
        List<Vector> finalMissionPointsList = new ArrayList<>();
        if (polygon == null) {
            return finalMissionPointsList;
        }

        //Create jts polygon for buffering
        Geometry bufferedPolygon = polygon.bufferOp(-10);
        Polygon bufferedPolygon1 = JtsHelper.JTSPolygon2Polygon(bufferedPolygon);
        strokePolygonLine(bufferedPolygon1, true);
        Geometry intersectionPolygon = bufferedPolygon1.bufferOp(1);

        //create root mission coordinates
        List<Vector> rootPolygonVertices = bufferedPolygon1.getVertices();
        if (convexWaypointsCalculator == null) {
            convexWaypointsCalculator = new ConvexWaypointsCalculator();
        }
        List<Vector> rootMissionCoordinates = convexWaypointsCalculator.calculateWaypoints(rootPolygonVertices, distanceBetweenLines, distanceToBoundaries, linesOffset, angle, startingPosition);
        fillPoint(rootMissionCoordinates.get(0));
        strokeLine(rootMissionCoordinates, false);

        //loop over mission linestrings intersecting with root polygon
        GeometryFactory gf = new GeometryFactory();
        List<List<Vector>> subMissionCoordinatesList = new ArrayList<>();
        boolean isIntersecting = true;
        int cutIndex = 0;
        while (isIntersecting) {
            System.out.println("\n\n------------- IS INTERSECTING -------------");
            //find intersections
            HashMap<Integer, LineStringDifference> intersections = new HashMap<>();
            int biggesDifference = 0;
            int ticker = 0;
            int ticker1 = 0;
            for (int i = 0; i < rootMissionCoordinates.size() - 1; i++) {
                Vector v = rootMissionCoordinates.get(i);
                Vector v1 = rootMissionCoordinates.get(i + 1);

                switch (ticker) {
                    case 0:
                        switch (ticker1) {
                            case 0:
                                if (v != null && v1 != null) {
                                    //create linestring
                                    LineString lineString = gf.createLineString(new Coordinate[]{new Coordinate(v.getX(), v.getY())
                                            , new Coordinate(v1.getX(), v1.getY())});

                                    Geometry difference = lineString.difference(intersectionPolygon);
                                    LineStringDifference lineStringDifference = new LineStringDifference(difference.getCoordinates(), false);
                                    if (lineStringDifference.getCoordinates().size() > 2) {
                                        Coordinate[] coords = new Coordinate[2];
                                        coords[0] = lineStringDifference.getCoordinates().get(0);
                                        coords[1] = lineStringDifference.getCoordinates().get(1);
                                        lineStringDifference.setCoordinates(coords);
                                    }
                                    if (difference.getCoordinates().length > 0) {
                                        intersections.put(i, lineStringDifference);
                                        if (difference.getCoordinates().length > biggesDifference) {
                                            biggesDifference = difference.getCoordinates().length;
                                        }
                                    }
                                }
                                ticker1 = 1;
                                break;

                            case 1:
                                if (v != null && v1 != null) {
                                    //create linestring
                                    LineString lineString = gf.createLineString(new Coordinate[]{new Coordinate(v.getX(), v.getY())
                                            , new Coordinate(v1.getX(), v1.getY())});

                                    Geometry difference = lineString.difference(intersectionPolygon);
                                    LineStringDifference lineStringDifference = new LineStringDifference(difference.getCoordinates(), false);
                                    if (lineStringDifference.getCoordinates().size() > 2) {
                                        Coordinate[] coords = new Coordinate[2];
                                        coords[0] = lineStringDifference.getCoordinates().get(lineStringDifference.getCoordinates().size() - 2);
                                        coords[1] = lineStringDifference.getCoordinates().get(lineStringDifference.getCoordinates().size() - 1);
                                        lineStringDifference.setCoordinates(coords);
                                    }
                                    if (difference.getCoordinates().length > 0) {
                                        intersections.put(i, lineStringDifference);
                                        if (difference.getCoordinates().length > biggesDifference) {
                                            biggesDifference = difference.getCoordinates().length;
                                        }
                                    }
                                }
                                ticker1 = 0;
                                break;
                        }
                        ticker = 1;
                        break;

                    case 1:
                        if (v != null && v1 != null) {
                            //create linestring
                            LineString lineString = gf.createLineString(new Coordinate[]{new Coordinate(v.getX(), v.getY())
                                    , new Coordinate(v1.getX(), v1.getY())});

                            Geometry difference = lineString.difference(intersectionPolygon);
                            LineStringDifference lineStringDifference = new LineStringDifference(difference.getCoordinates(), true);
                            if (difference.getCoordinates().length > 0) {
                                intersections.put(i, lineStringDifference);
                                if (difference.getCoordinates().length > biggesDifference) {
                                    biggesDifference = difference.getCoordinates().length;
                                }
                            }
                        }
                        ticker = 0;
                        break;
                }

            }

            //correct set of intersections
            System.out.println("\nintersections: ");
            intersections.entrySet().forEach(entry -> {
                System.out.println(entry.getKey() + " " + entry.getValue());
            });
            cutIndex = correctIntersection(subMissionCoordinatesList, rootMissionCoordinates, intersections, cutIndex);

            //TODO: delete test
            for (int i = 0; i < rootMissionCoordinates.size(); i++) {
                Vector v = rootMissionCoordinates.get(i);
                System.out.println("Root coord " + i + ": " + v);
            }

            isIntersecting = !intersections.isEmpty();

            /*if (test < 2) {
                break;
            }*/
        }


        return joinSubMissionList(subMissionCoordinatesList);
    }

    private int correctIntersection(List<List<Vector>> subMissionCoordinatesList, List<Vector> rootMissionCoordinates, HashMap<Integer, LineStringDifference> intersections, int cutIndex) {
        System.out.println("\n--- correcting intersection ---");
        List<Vector> mission = new ArrayList<>();
        boolean jumpOne = false;
        int ticker = 0;
        int newCutIndex = 0;
        // mission got cut in the middle
        if (!(cutIndex+1 < rootMissionCoordinates.size()))   {
            return 0;
        }
        else if (cutIndex != 0) {
            cutIndex++;
        }
        System.out.println("cut index: " + cutIndex);

        HashMap<Integer, Vector> swap = new HashMap<>();
        for (int j = cutIndex; j < rootMissionCoordinates.size(); j++)  {
            System.out.println("index " + j);
            if (rootMissionCoordinates.get(j) != null) {
                if (!intersections.containsKey(j)) {
                    if (!jumpOne) {
                        mission.add(rootMissionCoordinates.get(j));
                        rootMissionCoordinates.set(j, null);
                        System.out.println("not intersective");
                    }
                    //jumpOne = false;
                } else {
                    System.out.println("intersective");
                    LineStringDifference lineStringDifference = intersections.get(j);
                    List<Coordinate> coords = lineStringDifference.getCoordinates();

                    if (lineStringDifference.isSupportingLine()) {
                        System.out.println("lineString is supportive");
                        mission.add(rootMissionCoordinates.get(j));
                        rootMissionCoordinates.set(j, null);
                        newCutIndex = j;
                        break;
                    }

                    switch (ticker) {
                        case 0:
                            mission.add(rootMissionCoordinates.get(j));
                            Vector v = offsetCoordinate(JtsHelper.JTSCoordinate2Vector(coords.get(0)), rootMissionCoordinates.get(j));
                            Vector v1 = offsetCoordinate(JtsHelper.JTSCoordinate2Vector(coords.get(1)), rootMissionCoordinates.get(j+1));
                            mission.add(v);
                            //rootMissionCoordinates.set(j, v1);
                            swap.put(j, v1);
                            /*mission.add(JtsHelper.JTSCoordinate2Vector(coords.get(0)));
                            rootMissionCoordinates.set(j, JtsHelper.JTSCoordinate2Vector(coords.get(1)));*/
                            jumpOne = true;
                            ticker = 1;
                            break;

                        case 1:
                            Vector v2 = offsetCoordinate(JtsHelper.JTSCoordinate2Vector(coords.get(coords.size() - 1)), rootMissionCoordinates.get(j+1));
                            Vector v3 = offsetCoordinate(JtsHelper.JTSCoordinate2Vector(coords.get(0)), rootMissionCoordinates.get(j));
                            mission.add(v2);
                            mission.add(rootMissionCoordinates.get(j+1));
                            //rootMissionCoordinates.set(j+1, v3);
                            swap.put(j+1, v3);
                            /*mission.add(JtsHelper.JTSCoordinate2Vector(coords.get(coords.size() - 1)));
                            rootMissionCoordinates.set(j, JtsHelper.JTSCoordinate2Vector(coords.get(0)));*/

                            jumpOne = false;
                            ticker = 0;
                            break;
                    }

                    /*if (lineStringDifference.isSupportingLine()) {
                        System.out.println("lineString is supportive");

                        newCutIndex = j;
                        break;
                    }*/
                }
            }
        }

        //swap updated mission points in root list
        swap.entrySet().forEach(entry -> {
            rootMissionCoordinates.set(entry.getKey(), entry.getValue());
            System.out.println("swapping " + entry.getKey() + ": " + entry.getValue());
        });

        subMissionCoordinatesList.add(mission);

        if (test < 9) {
            strokeLine(mission, true);
            test++;
        }
        return newCutIndex;
    }

    private List<Vector> joinSubMissionList(List<List<Vector>> subMissionList)  {
        //TODO:
        for (int i = 1; i < subMissionList.size(); i++) {

        }

        return new ArrayList<>();
    }

    private Vector offsetCoordinate(Vector v1, Vector v2)    {
        Vector v = new Vector(v2.getX() - v1.getX(), v2.getY() - v1.getY());
        v.normalize();
        v.mult(4);
        v = new Vector(v.getX() + v1.getX(), v.getY() + v1.getY());
        return v;
    }
    //endregion

    //region test
    public void prepareForTesting(GraphicsContext c)    {
        this.graphicsContext = c;
    }

    private void fillPoint(Vector v) {
        graphicsContext.setFill(Color.RED);
        graphicsContext.fillOval(v.getX(), v.getY(), 5, 5);
    }

    private void strokePolygonLine(Polygon polygon, boolean random) {
        if (polygon != null)    {
            if (random) {
                graphicsContext.setStroke(Color.color(Math.random(), Math.random(), Math.random()));
                graphicsContext.setLineWidth(1.0);
            }
            else {
                graphicsContext.setStroke(Color.YELLOW);
                graphicsContext.setLineWidth(2.0);
            }
            for (int i = 0; i < polygon.getVertices().size()-1; i++)    {
                Vector v0 = polygon.getVertices().get(i);
                Vector v1 = polygon.getVertices().get(i+1);
                graphicsContext.strokeLine(v0.getX(), v0.getY(), v1.getX(), v1.getY());
            }

            if (polygon.getVertices().size() > 2) {
                Vector v0 = polygon.getVertices().get(polygon.getVertices().size()-1);
                Vector v1 = polygon.getVertices().get(0);
                graphicsContext.strokeLine(v0.getX(), v0.getY(), v1.getX(), v1.getY());
            }

        }
    }

    private void strokeLine(List<Vector> vectors, boolean random)    {
        if (vectors != null)    {
            if (random) {
                graphicsContext.setStroke(Color.color(Math.random(), Math.random(), Math.random()));
            }
            else {
                graphicsContext.setStroke(Color.YELLOW);
            }
            graphicsContext.setLineWidth(1.0);

            for (int i = 0; i < vectors.size()-1; i++)    {
                Vector v0 = vectors.get(i);
                Vector v1 = vectors.get(i+1);
                graphicsContext.strokeLine(v0.getX(), v0.getY(), v1.getX(), v1.getY());
            }
        }
    }
    //endregion
}
