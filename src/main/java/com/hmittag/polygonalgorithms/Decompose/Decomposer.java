package com.hmittag.polygonalgorithms.Decompose;

import com.hmittag.polygonalgorithms.DYN4J.Dyn4JHelper;
import com.hmittag.polygonalgorithms.Model.Polygon.Polygon;
import com.hmittag.polygonalgorithms.Model.Vector.Vector;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.decompose.Bayazit;

import java.util.ArrayList;
import java.util.List;

/**
 * Line sweep algorithm to decompose a polygonal nonconvex region (possibly with polygonal nonconvex holes)
 * into monotone regions with respect to a given sweep line.
 *
 */
public class Decomposer {
    //region decompose
    public List<Polygon> decompose(Polygon polygon) {
        List<Polygon> polygonParts = new ArrayList<>();
        Bayazit bayazit = new Bayazit();

        List<Convex> convexes = bayazit.decompose(Dyn4JHelper.VectorListToVector2List(polygon.getVertices()));
        for (Convex c : convexes) {
            org.dyn4j.geometry.Polygon dyn4jPolygon = (org.dyn4j.geometry.Polygon) c;
            Polygon p = Dyn4JHelper.dyn4JPolygonToPolygon(dyn4jPolygon);
            polygonParts.add(p);
        }
        return polygonParts;
    }

    //endregion
}
