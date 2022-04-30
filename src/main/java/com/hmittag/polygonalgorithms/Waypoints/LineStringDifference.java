package com.hmittag.polygonalgorithms.Waypoints;

import org.locationtech.jts.geom.Coordinate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LineStringDifference {
    //region constants

    //endregion

    //region fields
    private List<Coordinate> coordinates;
    private boolean isSupportingLine = false;
    //endregion

    //region constructor

    public LineStringDifference(Coordinate[] coordinates, boolean isSupportingLine) {
        this.coordinates = Arrays.asList(coordinates);
        this.isSupportingLine = isSupportingLine;
    }

    //endregion


    public List<Coordinate> getCoordinates() {
        return coordinates;
    }
    public void setCoordinates(Coordinate[] coordinates) {
        this.coordinates = Arrays.asList(coordinates);
    }

    public boolean isSupportingLine() {
        return isSupportingLine;
    }
    public void setSupportingLine(boolean supportingLine) {
        isSupportingLine = supportingLine;
    }
}
