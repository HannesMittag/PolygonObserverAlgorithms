module com.hmittag.polygonalgorithms {
    requires javafx.controls;
    requires javafx.fxml;
    requires org.locationtech.jts;
    requires org.dyn4j;


    opens com.hmittag.polygonalgorithms to javafx.fxml;
    exports com.hmittag.polygonalgorithms;
}