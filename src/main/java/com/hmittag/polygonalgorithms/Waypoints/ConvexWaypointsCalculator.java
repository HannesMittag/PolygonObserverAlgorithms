package com.hmittag.polygonalgorithms.Waypoints;

import com.hmittag.polygonalgorithms.Model.Vector.Vector;

import java.util.ArrayList;
import java.util.List;

public class ConvexWaypointsCalculator {

    private double offset = 0.0;

    public List<Vector> calculateWaypointsForLongestSide(List<Vector> polygonCoordinates, double distanceBetweenLines, double distanceToBoundaries, double linesOffset, StartingPosition startingPosition) {
        List<double[]> coordinates = new ArrayList<>();
        for (Vector v : polygonCoordinates) {
            coordinates.add(new double[] {v.getX(), v.getY()});
        }
        List<double[]> offsetPolygonCoordinates = calculatePolygonCoordinatesWithOffset(coordinates, distanceToBoundaries);

        //TODO: delete test
        /*List<Vector> test = new ArrayList<>();
        for (double[] p : offsetPolygonCoordinates) {
            test.add(new Vector(p[0], p[1]));
        }

        return test;*/

        /*List<double[]> coordinates = new ArrayList<>();
        for (Vector v : polygonCoordinates) {
            coordinates.add(new double[] {v.getX(), v.getY()});
        }*/

        double degree = calculateDegreeOfLongestSide(offsetPolygonCoordinates);
        //System.out.println("angle: " + degree);

        return calculateWaypoints(polygonCoordinates, distanceBetweenLines, distanceToBoundaries, linesOffset, degree, startingPosition);
    }

    public List<Vector> calculateWaypoints(List<Vector> polygonCoordinates, double distanceBetweenLines, double distanceToBoundaries, double linesOffset, double degree, StartingPosition startingPosition) {

        List<double[]> coords = new ArrayList<>();
        for (Vector v : polygonCoordinates) {
            coords.add(new double[]  {v.getX(), v.getY()});
        }

        List<double[]> offsetPolygonCoordinates = calculatePolygonCoordinatesWithOffset(coords, distanceToBoundaries);

        StartingPosition rotatedStartingPosition = getRotatedStartingPosition(degree, startingPosition);

        double[] boundingBox = calculateBoundingBox(offsetPolygonCoordinates);
        //double[] boundingBox = calculateBoundingBox(polygonCoordinates);
        double midpointLatitude = (boundingBox[0] + boundingBox[1]) / 2;
        double midpointLongitude = (boundingBox[2] + boundingBox[3]) / 2;
        List<double[]> rotatedPolygonCoordinates = new ArrayList<>();

        for (double[] polygonCoordinate : offsetPolygonCoordinates) {
            double[] newPolygonCoordinate = new double[] {polygonCoordinate[0] - midpointLongitude, polygonCoordinate[1] - midpointLatitude};
            //double[] newPolygonCoordinate = new double[] {polygonCoordinate[0] - midpointLatitude, polygonCoordinate[1] - midpointLongitude};
            newPolygonCoordinate = rotateCoordinate(newPolygonCoordinate, degree);
            rotatedPolygonCoordinates.add(newPolygonCoordinate);
        }

        double[] rotatedBoundingBox = calculateBoundingBox(rotatedPolygonCoordinates);

        this.offset = calculateOffset(rotatedBoundingBox, degree, new double[]{midpointLongitude, midpointLatitude}, true);

        List<double[]> rotatedWaypoints = calculateWaypoints(rotatedPolygonCoordinates, distanceBetweenLines, linesOffset, rotatedBoundingBox, rotatedStartingPosition);
        List<double[]> waypoints = new ArrayList<>();

        for (double[] rotatedWaypoint : rotatedWaypoints) {
            double[] waypoint = rotateCoordinate(rotatedWaypoint, degree);
            //double[] waypoint = rotateCoordinate(rotatedWaypoint, 0);
            //double[] movedWaypoint = new double[] {waypoint[1] + midpointLatitude, waypoint[0] + midpointLongitude};

            double[] movedWaypoint = new double[] {waypoint[0] + midpointLongitude, waypoint[1] + midpointLatitude};
            waypoints.add(movedWaypoint);
        }

        // Check if first or last distance is smaller than 1.5 meter to prevent "Waypoint distance too close" error
        if (waypoints.size() >= 2) {
            double distanceFirstTwoPoints = distanceBetweenTwoPoints(waypoints.get(0), waypoints.get(1));
            /*if (distanceFirstTwoPoints < 1.5) {
                waypoints.remove(0);
            }*/
            if (distanceFirstTwoPoints < 1.5) {
                waypoints.remove(0);
            }
        }
        if (waypoints.size() >= 2) {
            double distanceLastTwoPoints = distanceBetweenTwoPoints(waypoints.get(waypoints.size() - 2), waypoints.get(waypoints.size() - 1));
            /*if (distanceLastTwoPoints < 1.5) {
                waypoints.remove(waypoints.size() - 1);
            }*/
            if (distanceLastTwoPoints < 1.5) {
                waypoints.remove(waypoints.size() - 1);
            }
        }

        List<Vector> ret = new ArrayList<>();
        for (double[] v : waypoints)    {
            ret.add(new Vector(v[0], v[1]));
        }

        return ret;
    }

    public List<double[]> calculateWaypoints(List<double[]> polygonCoordinates, double distanceBetweenLines, double linesOffset, double[] boundingBox, StartingPosition startingPosition) {
        // Make sure the offset is smaller than distance between lines
        double firstOffsetMeter = linesOffset;
        if (firstOffsetMeter >= distanceBetweenLines || firstOffsetMeter <= (distanceBetweenLines * -1.0)) {
            firstOffsetMeter = firstOffsetMeter % distanceBetweenLines;
        }

        /*double offset = distanceBetweenLines * this.offset;
        double firstOffset = firstOffsetMeter * this.offset;*/
        double offset = distanceBetweenLines;
        double firstOffset = firstOffsetMeter;
        double actualOffset;
        boolean startBottom;
        boolean startRight;
        boolean offsetNotMaximum = true;
        List<double[]> waypoints = new ArrayList<>();

        if (startingPosition == StartingPosition.BOTTOM_RIGHT || startingPosition == StartingPosition.BOTTOM_LEFT) {
            startBottom = true;
        } else {
            startBottom = false;
        }

        if (startingPosition == StartingPosition.BOTTOM_RIGHT || startingPosition == StartingPosition.TOP_RIGHT) {
            startRight = true;
            //actualOffset = boundingBox[3] - firstOffset;
            actualOffset = boundingBox[3] - firstOffset;
        } else {
            startRight = false;
            //actualOffset = boundingBox[2] + firstOffset;
            actualOffset = boundingBox[2] + firstOffset;
        }

        while (offsetNotMaximum) {
            if (startRight) {
                actualOffset -= offset;
            } else {
                actualOffset += offset;
            }

            List<double[]> intersections = new ArrayList<>();

            for (int i = 1; i < polygonCoordinates.size(); i++) {
                double[] point1 = polygonCoordinates.get(i - 1);
                double[] point2 = polygonCoordinates.get(i);

                double[] intersection = calculateIntersection(point1[0], point1[1], point2[0]
                        , point2[1], actualOffset, boundingBox[0], actualOffset, boundingBox[1]);

                if (intersection != null) {
                    intersections.add(intersection);
                }
            }

            double[] waypointBottom = getMinLat(intersections);
            double[] waypointTop = getMaxLat(intersections);
            if (startBottom) {
                if (waypointBottom != null) {
                    waypoints.add(waypointBottom);
                }
                if (waypointTop != null) {
                    waypoints.add(waypointTop);
                }
                startBottom = !startBottom;
            } else {
                if (waypointTop != null) {
                    waypoints.add(waypointTop);
                }
                if (waypointBottom != null) {
                    waypoints.add(waypointBottom);
                }
                startBottom = !startBottom;
            }

            if (startRight) {
                if (actualOffset <= boundingBox[2]) {
                    offsetNotMaximum = false;
                }
            } else {
                if (actualOffset >= boundingBox[3]) {
                    offsetNotMaximum = false;
                }
            }
        }

        return waypoints;
    }

    public List<double[]> calculatePolygonCoordinatesWithOffset(List<double[]> polygonCoordinates, double offsetInMeters) {
        List<double[]> newPolygonCoordinates = new ArrayList<>();

        List<double[]> firstLine1 = new ArrayList<>();
        firstLine1.add(polygonCoordinates.get(polygonCoordinates.size() - 2));
        firstLine1.add(polygonCoordinates.get(0));

        List<double[]> firstLine2 = new ArrayList<>();
        firstLine2.add(polygonCoordinates.get(0));
        firstLine2.add(polygonCoordinates.get(1));

        boolean isPolygonClockwise = isPolygonClockwise(polygonCoordinates);

        double[] firstNewPolygonCoordinate;

        if (isPolygonClockwise) {
            firstNewPolygonCoordinate = calculateLinesIntersectionWithOffset(firstLine1, firstLine2, offsetInMeters * -1);
        } else {
            firstNewPolygonCoordinate = calculateLinesIntersectionWithOffset(firstLine1, firstLine2, offsetInMeters);
        }

        newPolygonCoordinates.add(firstNewPolygonCoordinate);

        for (int i = 1; i < polygonCoordinates.size() - 2; i++) {

            List<double[]> line1 = new ArrayList<>();
            line1.add(polygonCoordinates.get(i - 1));
            line1.add(polygonCoordinates.get(i));

            List<double[]> line2 = new ArrayList<>();
            line2.add(polygonCoordinates.get(i));
            line2.add(polygonCoordinates.get(i + 1));

            double[] newPolygonCoordinate;

            if (isPolygonClockwise) {
                newPolygonCoordinate = calculateLinesIntersectionWithOffset(line1, line2, offsetInMeters * -1);
            } else {
                newPolygonCoordinate = calculateLinesIntersectionWithOffset(line1, line2, offsetInMeters);
            }

            newPolygonCoordinates.add(newPolygonCoordinate);
        }

        List<double[]> lastLine1 = new ArrayList<>();
        lastLine1.add(polygonCoordinates.get(polygonCoordinates.size() - 3));
        lastLine1.add(polygonCoordinates.get(polygonCoordinates.size() - 2));

        List<double[]> lastLine2 = new ArrayList<>();
        lastLine2.add(polygonCoordinates.get(polygonCoordinates.size() - 2));
        lastLine2.add(polygonCoordinates.get(0));

        double[] lastNewPolygonCoordinate;

        if (isPolygonClockwise) {
            lastNewPolygonCoordinate = calculateLinesIntersectionWithOffset(lastLine1, lastLine2, offsetInMeters * -1);
        } else {
            lastNewPolygonCoordinate = calculateLinesIntersectionWithOffset(lastLine1, lastLine2, offsetInMeters);
        }

        newPolygonCoordinates.add(lastNewPolygonCoordinate);
        newPolygonCoordinates.add(newPolygonCoordinates.get(0));

        return newPolygonCoordinates;
    }

    private double[] calculateBoundingBox(List<double[]> polygonCoordinates) {

        double minLatitude = polygonCoordinates.get(0)[1];
        double maxLatitude = polygonCoordinates.get(0)[1];
        double minLongitude = polygonCoordinates.get(0)[0];
        double maxLongitude = polygonCoordinates.get(0)[0];

        for (double[] polygonCoordinate : polygonCoordinates) {
            if (polygonCoordinate[0] < minLongitude) {
                minLongitude = polygonCoordinate[0];
            } else if (polygonCoordinate[0] > maxLongitude) {
                maxLongitude = polygonCoordinate[0];
            }

            if (polygonCoordinate[1] < minLatitude) {
                minLatitude = polygonCoordinate[1];
            } else if (polygonCoordinate[1] > maxLatitude) {
                maxLatitude = polygonCoordinate[1];
            }
        }
        return new double[] {minLatitude, maxLatitude, minLongitude, maxLongitude};
    }

    private double[] getMaxLat(List<double[]> pointList) {
        if (pointList.size() > 0) {
            double[] maxLatLng = pointList.get(0);

            for (double[] point : pointList) {
                if (point[1] > maxLatLng[1]) {
                    maxLatLng = point;
                }
            }

            return maxLatLng;
        } else {
            return null;
        }
    }

    private double[] getMinLat(List<double[]> pointList) {
        if (pointList.size() > 0) {
            double[] minLatLng = pointList.get(0);

            for (double[] point : pointList) {
                if (point[1] < minLatLng[1]) {
                    minLatLng = point;
                }
            }

            return minLatLng;
        } else {
            return null;
        }
    }

    private double[] calculateIntersection(double x1, double y1, double x2, double y2
            , double x3, double y3, double x4, double y4) {

        if (x1 == x2 && x3 == x4) {
            return null;
        } else if (x1 == x2) {
            if ((x1 > x3 && x1 < x4) || (x1 < x3 && x1 > x4)) {
                double m = (y4 - y3) / (x4 - x3);
                double c = y3 - m * x3;
                double y = m * x1 + c;
                //return new double[]{y, x1};
                return new double[]{x1, y};
            } else {
                return null;
            }
        } else if (x3 == x4) {
            if ((x3 > x1 && x3 < x2) || (x3 < x1 && x3 > x2)) {
                double m = (y2 - y1) / (x2 - x1);
                double c = y1 - m * x1;
                double y = m * x3 + c;
                //return new double[]{y, x3};
                return new double[]{x3, y};
            } else {
                return null;
            }
        } else {
            double a1 = x2 - x1;
            double b1 = x4 - x3;
            double c1 = x3 - x1;
            double a2 = y2 - y1;
            double b2 = y4 - y3;
            double c2 = y3 - y1;
            double s0 = (c1 * b2 - c2 * b1) / (a1 * b2 - a2 * b1);
            double t0 = (a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);

            if (s0 >= 0 && s0 <= 1 && t0 >= 0 && t0 <= 1) {

                double x = x1 + s0 * (x2 - x1);
                double y = y1 + s0 * (y2 - y1);

                //return new double[]{y, x};
                return new double[]{x, y};
            } else {
                return null;
            }
        }
    }

    private double[] calculateLineIntersection(double x1, double y1, double x2, double y2
            , double x3, double y3, double x4, double y4) {

        // Zaehler
        double zx = (x1 * y2 - y1 * x2)*(x3-x4) - (x1 - x2) * (x3 * y4 - y3 * x4);
        double zy = (x1 * y2 - y1 * x2)*(y3-y4) - (y1 - y2) * (x3 * y4 - y3 * x4);

        // Nenner
        double n = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        // Koordinaten des Schnittpunktes
        double x = zx/n;
        double y = zy/n;

        return new double[]{x, y};
    }

    private double[] rotateCoordinate(double[] coordinate, double degree) {

        double radians = Math.toRadians(degree);

        double longitude = coordinate[1] * Math.cos(radians) - coordinate[0] * Math.sin(radians);
        double latitude = coordinate[1] * Math.sin(radians) + coordinate[0] * Math.cos(radians);

        return new double[] {longitude, latitude};
    }

    public double calculateDegreeOfLongestSide(List<double[]> polygonCoordinates) {

        // Calculate longest side
        double[] point1 = polygonCoordinates.get(0);
        double[] point2 = polygonCoordinates.get(0);
        double maxLength = 0;

        for (int i = 1; i < polygonCoordinates.size(); i++) {
            double length = distanceBetweenTwoPoints(polygonCoordinates.get(i - 1), polygonCoordinates.get(i));

            if (length > maxLength) {
                maxLength = length;
                point1 = polygonCoordinates.get(i - 1);
                point2 = polygonCoordinates.get(i);
            }
        }

        // Calculate degree
        System.out.println(point1);
        System.out.println(point2);
        double m = (point2[1] - point1[1]) / (point2[0] - point1[0]);
        double degree = Math.atan(m);
        degree = Math.toDegrees(degree);

        return degree;
    }

    private ConvexWaypointsCalculator.StartingPosition getRotatedStartingPosition(double degree, StartingPosition startingPosition) {
        if (degree < 0) {
            degree = 360 + degree;
        }
        if (degree > 315 || degree <= 45) {
            switch (startingPosition) {
                case BOTTOM_RIGHT:
                    return StartingPosition.TOP_LEFT;
                case BOTTOM_LEFT:
                    return StartingPosition.BOTTOM_LEFT;
                case TOP_RIGHT:
                    return StartingPosition.TOP_RIGHT;
                case TOP_LEFT:
                    return StartingPosition.BOTTOM_RIGHT;
            }
        } else if (degree > 45 && degree <= 135) {
            switch (startingPosition) {
                case BOTTOM_RIGHT:
                    return StartingPosition.BOTTOM_LEFT;
                case BOTTOM_LEFT:
                    return StartingPosition.BOTTOM_RIGHT;
                case TOP_RIGHT:
                    return StartingPosition.TOP_LEFT;
                case TOP_LEFT:
                    return StartingPosition.TOP_RIGHT;
            }
        } else if (degree > 135 && degree <= 225) {
            switch (startingPosition) {
                case BOTTOM_RIGHT:
                    return StartingPosition.BOTTOM_RIGHT;
                case BOTTOM_LEFT:
                    return StartingPosition.TOP_RIGHT;
                case TOP_RIGHT:
                    return StartingPosition.BOTTOM_LEFT;
                case TOP_LEFT:
                    return StartingPosition.TOP_LEFT;
            }
        } else {
            switch (startingPosition) {
                case BOTTOM_RIGHT:
                    return StartingPosition.TOP_RIGHT;
                case BOTTOM_LEFT:
                    return StartingPosition.TOP_LEFT;
                case TOP_RIGHT:
                    return StartingPosition.BOTTOM_RIGHT;
                case TOP_LEFT:
                    return StartingPosition.BOTTOM_LEFT;
            }
        }
        return StartingPosition.BOTTOM_RIGHT;
    }

    private double calculateOffset(double[] boundingBox, double degree, double[] midpoint, boolean bottom)  {

        double[] boundingBoxPoint1;
        double[] boundingBoxPoint2;

        if (bottom) {
            boundingBoxPoint1 = new double[]{boundingBox[0], boundingBox[2]};
            boundingBoxPoint2 = new double[]{boundingBox[0], boundingBox[3]};
        } else {
            boundingBoxPoint1 = new double[]{boundingBox[1], boundingBox[2]};
            boundingBoxPoint2 = new double[]{boundingBox[1], boundingBox[3]};
        }

        boundingBoxPoint1 = rotateCoordinate(boundingBoxPoint1, degree);
        boundingBoxPoint2 = rotateCoordinate(boundingBoxPoint2, degree);

        boundingBoxPoint1 = new double[]{boundingBoxPoint1[0] + midpoint[0], boundingBoxPoint1[1] + midpoint[1]};
        boundingBoxPoint2 = new double[]{boundingBoxPoint2[0] + midpoint[0], boundingBoxPoint2[1] + midpoint[1]};

        //double meterDistance = calculateDistanceInMeters(boundingBoxPoint1, boundingBoxPoint2);
        double coordinateDistance = Math.sqrt(Math.pow((boundingBoxPoint1[0] - boundingBoxPoint2[0]), 2) + Math.pow((boundingBoxPoint1[1] - boundingBoxPoint2[1]), 2));
        return distanceBetweenTwoPoints(boundingBoxPoint1,  boundingBoxPoint2);
        //return coordinateDistance;
    }

    private List<double[]> moveLineParallelWithOffset(List<double[]> line, double offsetInMeters) {

        double[] point1 = line.get(0);
        double[] point2 = line.get(1);

        // Calculate perpendicular vector of line and normalize it
        double[] perpendicularVector = new double[]{point2[0] - point1[0], (point2[1] - point1[1]) * -1};
        double vectorLength = Math.sqrt(Math.pow(perpendicularVector[0], 2) + Math.pow(perpendicularVector[1], 2));
        perpendicularVector = new double[]{perpendicularVector[0] / vectorLength, perpendicularVector[1] / vectorLength};

        // Calculate one meter offset
        double[] pointToCalculateOffset = new double[]{point1[0] + perpendicularVector[0], point1[1] + perpendicularVector[1]};
        double meterDistance = distanceBetweenTwoPoints(point1, pointToCalculateOffset);
        double coordinateDistance = Math.sqrt(Math.pow((point1[0] - pointToCalculateOffset[0]), 2) + Math.pow((point1[1] - pointToCalculateOffset[1]), 2));
        double oneMeterOffset = coordinateDistance / meterDistance;

        // Calculate the new moved line
        List<double[]> movedLine = new ArrayList<>();
        movedLine.add(new double[]{point1[0] + (perpendicularVector[0] * (offsetInMeters)), point1[1] + (perpendicularVector[1] * (offsetInMeters))});
        movedLine.add(new double[]{point2[0] + (perpendicularVector[0] * (offsetInMeters)), point2[1] + (perpendicularVector[1] * (offsetInMeters))});

        return movedLine;
    }

    private double[] calculateLinesIntersectionWithOffset(List<double[]> line1, List<double[]> line2, double offsetInMeters) {
        List<double[]> movedLine1 = moveLineParallelWithOffset(line1, offsetInMeters);
        List<double[]> movedLine2 = moveLineParallelWithOffset(line2, offsetInMeters);
        return calculateLineIntersection(movedLine1.get(0)[0], movedLine1.get(0)[1], movedLine1.get(1)[0], movedLine1.get(1)[1],
                movedLine2.get(0)[0], movedLine2.get(0)[1], movedLine2.get(1)[0], movedLine2.get(1)[1]);
    }

    private boolean isPolygonClockwise(List<double[]> polygon) {
        double sum = 0;
        for (int i = 0; i < polygon.size() - 1; i++) {
            double[] polygonCoordinate1 = polygon.get(i);
            double[] polygonCoordinate2 = polygon.get(i + 1);
            sum += (polygonCoordinate2[0] - polygonCoordinate1[0]) * (polygonCoordinate2[1] + polygonCoordinate1[1]);
        }
        if (sum < 0) {
            return false;
        } else {
            return true;
        }
    }

    public static double distanceBetweenTwoPoints(double[] p0, double[] p1)  {
        return Math.sqrt(Math.pow(p1[0] - p0[0], 2) + Math.pow(p1[1] - p0[1], 2));
    }

    public enum StartingPosition    {
        BOTTOM_RIGHT,
        BOTTOM_LEFT,
        TOP_RIGHT,
        TOP_LEFT
    }
}
