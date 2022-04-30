package com.hmittag.polygonalgorithms;

import com.hmittag.polygonalgorithms.View.CanvasPrinter;
import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.stage.Stage;

import java.io.IOException;

public class HelloApplication extends Application {
    //region constants
    public static final int [] WINDOW_DIMS = new int[]    {1000, 700};
    //endregion

    @Override
    public void start(Stage stage) throws IOException {
        Group root = new Group();
        Canvas canvas = new Canvas(WINDOW_DIMS[0], WINDOW_DIMS[1]);
        root.getChildren().add(canvas);

        CanvasPrinter canvasPrinter = new CanvasPrinter(canvas);
        canvasPrinter.setupCanvas();

        Scene scene = new Scene(root, WINDOW_DIMS[0], WINDOW_DIMS[1]);
        stage.setTitle("Polygon Algorithms");
        stage.setScene(scene);
        stage.show();
    }

    public static void main(String[] args) {
        launch();
    }
}