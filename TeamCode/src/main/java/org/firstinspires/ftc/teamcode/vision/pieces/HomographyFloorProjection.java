package org.firstinspires.ftc.teamcode.vision.pieces;

/**
 * Planar homography from image pixels {@code (u, v)} to robot-floor inches.
 *
 * <pre>
 * [x']   [h00 h01 h02] [u]
 * [y'] = [h10 h11 h12] [v]
 * [w ]   [h20 h21 h22] [1]
 * x = x' / w,  y = y' / w
 * </pre>
 *
 * <p>There is no measured camera mount in this class. {@link #unconfigured()}
 * refuses to project. Coefficients come from {@link #fromCoefficients(double[])}
 * or from a measured set of pixel and floor pairs. {@link #solve(double[][], double[][])}
 * is the four-point exact fit, an 8-by-8 elimination with {@code h22} fixed at 1.
 * {@link #calibrate(double[][], double[][])} accepts four or more pairs. Extra
 * pairs use a normalized least-squares fit of those same eight coefficients.
 * Both report residuals through {@link Calibration}. This is not a camera library.
 */
public final class HomographyFloorProjection implements FloorProjection {

    private static final double PIVOT_EPSILON = 1.0e-10;
    private static final double WEIGHT_EPSILON = 1.0e-9;

    private final double[] h;
    private final boolean configured;

    private HomographyFloorProjection(double[] h, boolean configured) {
        this.h = h;
        this.configured = configured;
    }

    public static HomographyFloorProjection unconfigured() {
        return new HomographyFloorProjection(new double[9], false);
    }

    /**
     * Row-major 3 by 3 coefficients. A non-finite or wrong-length array is
     * treated as not configured.
     */
    public static HomographyFloorProjection fromCoefficients(double[] rowMajor) {
        if (rowMajor == null || rowMajor.length != 9) {
            return unconfigured();
        }
        double[] copy = new double[9];
        for (int i = 0; i < 9; i++) {
            if (!Double.isFinite(rowMajor[i])) {
                return unconfigured();
            }
            copy[i] = rowMajor[i];
        }
        return new HomographyFloorProjection(copy, true);
    }

    /**
     * Four image pixels and the robot-floor inches where those pieces sat.
     * Returns an unconfigured projection when the points do not determine a
     * homography.
     */
    public static HomographyFloorProjection solve(double[][] imagePixels, double[][] floorInches) {
        if (imagePixels == null || floorInches == null || imagePixels.length != 4 || floorInches.length != 4) {
            return unconfigured();
        }
        double[][] matrix = new double[8][9];
        for (int i = 0; i < 4; i++) {
            if (imagePixels[i] == null || floorInches[i] == null
                    || imagePixels[i].length < 2 || floorInches[i].length < 2) {
                return unconfigured();
            }
            double u = imagePixels[i][0];
            double v = imagePixels[i][1];
            double x = floorInches[i][0];
            double y = floorInches[i][1];
            if (!Double.isFinite(u) || !Double.isFinite(v) || !Double.isFinite(x) || !Double.isFinite(y)) {
                return unconfigured();
            }
            int row = i * 2;
            matrix[row][0] = u;
            matrix[row][1] = v;
            matrix[row][2] = 1.0;
            matrix[row][6] = -x * u;
            matrix[row][7] = -x * v;
            matrix[row][8] = x;
            matrix[row + 1][3] = u;
            matrix[row + 1][4] = v;
            matrix[row + 1][5] = 1.0;
            matrix[row + 1][6] = -y * u;
            matrix[row + 1][7] = -y * v;
            matrix[row + 1][8] = y;
        }
        if (!eliminate(matrix)) {
            return unconfigured();
        }
        double[] coefficients = new double[9];
        for (int i = 0; i < 8; i++) {
            coefficients[i] = matrix[i][8];
        }
        coefficients[8] = 1.0;
        return fromCoefficients(coefficients);
    }

    /**
     * Fits a homography to {@code N} measured pairs, {@code N >= 4}.
     * Four pairs use {@link #solve(double[][], double[][])}. More than four
     * minimize squared algebraic error on normalized coordinates, then the
     * residual is the floor-inch error of that fit.
     * Fewer than four pairs, or a singular fit, are not configured.
     * An unconfigured result has non-finite residuals so a zero error is not implied.
     */
    public static Calibration calibrate(double[][] imagePixels, double[][] floorInches) {
        if (!usablePairs(imagePixels, floorInches) || imagePixels.length < 4) {
            return Calibration.notConfigured(imagePixels == null ? 0 : imagePixels.length);
        }
        HomographyFloorProjection projection = imagePixels.length == 4
                ? solve(imagePixels, floorInches)
                : solveOverdetermined(imagePixels, floorInches);
        return residuals(projection, imagePixels, floorInches);
    }

    private static HomographyFloorProjection solveOverdetermined(double[][] imagePixels, double[][] floorInches) {
        Similarity image = normalize(imagePixels);
        Similarity floor = normalize(floorInches);
        if (image == null || floor == null) {
            return unconfigured();
        }
        int sampleCount = imagePixels.length;
        double[][] rows = new double[sampleCount * 2][8];
        double[] rhs = new double[sampleCount * 2];
        for (int i = 0; i < sampleCount; i++) {
            double u = image.applyX(imagePixels[i][0]);
            double v = image.applyY(imagePixels[i][1]);
            double x = floor.applyX(floorInches[i][0]);
            double y = floor.applyY(floorInches[i][1]);
            int row = i * 2;
            rows[row][0] = u;
            rows[row][1] = v;
            rows[row][2] = 1.0;
            rows[row][6] = -x * u;
            rows[row][7] = -x * v;
            rhs[row] = x;
            rows[row + 1][3] = u;
            rows[row + 1][4] = v;
            rows[row + 1][5] = 1.0;
            rows[row + 1][6] = -y * u;
            rows[row + 1][7] = -y * v;
            rhs[row + 1] = y;
        }
        double[] coefficients = leastSquares(rows, rhs);
        if (coefficients == null) {
            return unconfigured();
        }
        double[] normalized = new double[] {
                coefficients[0], coefficients[1], coefficients[2],
                coefficients[3], coefficients[4], coefficients[5],
                coefficients[6], coefficients[7], 1.0
        };
        double[] homography = multiply(multiply(floor.inverseMatrix(), normalized), image.matrix());
        if (!Double.isFinite(homography[8]) || Math.abs(homography[8]) < WEIGHT_EPSILON) {
            return unconfigured();
        }
        for (int i = 0; i < 9; i++) {
            homography[i] /= homography[8];
        }
        return fromCoefficients(homography);
    }

    private static Calibration residuals(
            HomographyFloorProjection projection,
            double[][] imagePixels,
            double[][] floorInches) {
        if (projection == null || !projection.isConfigured()) {
            return Calibration.notConfigured(imagePixels.length);
        }
        double sumSquared = 0.0;
        double max = 0.0;
        for (int i = 0; i < imagePixels.length; i++) {
            FloorPoint point = projection.project(imagePixels[i][0], imagePixels[i][1]);
            if (point == null) {
                return Calibration.notConfigured(imagePixels.length);
            }
            double dx = point.getXForwardInches() - floorInches[i][0];
            double dy = point.getYLeftInches() - floorInches[i][1];
            double squared = dx * dx + dy * dy;
            if (!Double.isFinite(squared)) {
                return Calibration.notConfigured(imagePixels.length);
            }
            sumSquared += squared;
            double error = Math.sqrt(squared);
            if (error > max) {
                max = error;
            }
        }
        return new Calibration(
                projection,
                Math.sqrt(sumSquared / imagePixels.length),
                max,
                imagePixels.length);
    }

    private static boolean usablePairs(double[][] imagePixels, double[][] floorInches) {
        if (imagePixels == null || floorInches == null || imagePixels.length != floorInches.length) {
            return false;
        }
        for (int i = 0; i < imagePixels.length; i++) {
            if (imagePixels[i] == null || floorInches[i] == null
                    || imagePixels[i].length < 2 || floorInches[i].length < 2) {
                return false;
            }
            if (!Double.isFinite(imagePixels[i][0]) || !Double.isFinite(imagePixels[i][1])
                    || !Double.isFinite(floorInches[i][0]) || !Double.isFinite(floorInches[i][1])) {
                return false;
            }
        }
        return true;
    }

    /** Normal equations for an 8-column design. Singular fits return null. */
    private static double[] leastSquares(double[][] rows, double[] rhs) {
        int columns = 8;
        double[][] normal = new double[columns][columns + 1];
        for (int row = 0; row < columns; row++) {
            for (int col = 0; col < columns; col++) {
                double sum = 0.0;
                for (int sample = 0; sample < rhs.length; sample++) {
                    sum += rows[sample][row] * rows[sample][col];
                }
                normal[row][col] = sum;
            }
            double sum = 0.0;
            for (int sample = 0; sample < rhs.length; sample++) {
                sum += rows[sample][row] * rhs[sample];
            }
            normal[row][columns] = sum;
        }
        if (!eliminate(normal)) {
            return null;
        }
        double[] solution = new double[columns];
        for (int row = 0; row < columns; row++) {
            solution[row] = normal[row][columns];
        }
        return solution;
    }

    private static Similarity normalize(double[][] points) {
        double centerX = 0.0;
        double centerY = 0.0;
        for (int i = 0; i < points.length; i++) {
            centerX += points[i][0];
            centerY += points[i][1];
        }
        centerX /= points.length;
        centerY /= points.length;
        double meanDistance = 0.0;
        for (int i = 0; i < points.length; i++) {
            meanDistance += Math.hypot(points[i][0] - centerX, points[i][1] - centerY);
        }
        meanDistance /= points.length;
        if (!Double.isFinite(meanDistance)) {
            return null;
        }
        double scale = meanDistance < 1.0e-9 ? 1.0 : Math.sqrt(2.0) / meanDistance;
        if (!Double.isFinite(scale) || scale <= 0.0) {
            return null;
        }
        return new Similarity(scale, centerX, centerY);
    }

    private static double[] multiply(double[] left, double[] right) {
        double[] product = new double[9];
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                product[row * 3 + col] = left[row * 3] * right[col]
                        + left[row * 3 + 1] * right[3 + col]
                        + left[row * 3 + 2] * right[6 + col];
            }
        }
        return product;
    }

    @Override
    public boolean isConfigured() {
        return configured;
    }

    @Override
    public FloorPoint project(double imageX, double imageY) {
        if (!configured || !Double.isFinite(imageX) || !Double.isFinite(imageY)) {
            return null;
        }
        double weight = h[6] * imageX + h[7] * imageY + h[8];
        if (!Double.isFinite(weight) || Math.abs(weight) < WEIGHT_EPSILON) {
            return null;
        }
        double x = (h[0] * imageX + h[1] * imageY + h[2]) / weight;
        double y = (h[3] * imageX + h[4] * imageY + h[5]) / weight;
        return FloorPoint.of(x, y);
    }

    /** Gauss-Jordan elimination. The last column is the right-hand side. */
    private static boolean eliminate(double[][] matrix) {
        int n = matrix.length;
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(matrix[row][col]) > Math.abs(matrix[pivot][col])) {
                    pivot = row;
                }
            }
            if (Math.abs(matrix[pivot][col]) < PIVOT_EPSILON) {
                return false;
            }
            double[] swap = matrix[col];
            matrix[col] = matrix[pivot];
            matrix[pivot] = swap;
            double scale = matrix[col][col];
            for (int c = col; c <= n; c++) {
                matrix[col][c] /= scale;
            }
            for (int row = 0; row < n; row++) {
                if (row == col) {
                    continue;
                }
                double factor = matrix[row][col];
                for (int c = col; c <= n; c++) {
                    matrix[row][c] -= factor * matrix[col][c];
                }
            }
        }
        for (int row = 0; row < n; row++) {
            if (!Double.isFinite(matrix[row][n])) {
                return false;
            }
        }
        return true;
    }

    /**
     * A fitted floor map plus the inch error on the samples that produced it.
     * {@link #getRmsErrorInches()} is the root-mean-square of the point errors.
     * {@link #getMaxErrorInches()} is the worst sample. Both are non-finite
     * when the fit was rejected.
     */
    public static final class Calibration {
        private final HomographyFloorProjection projection;
        private final double rmsErrorInches;
        private final double maxErrorInches;
        private final int sampleCount;

        private Calibration(
                HomographyFloorProjection projection,
                double rmsErrorInches,
                double maxErrorInches,
                int sampleCount) {
            this.projection = projection;
            this.rmsErrorInches = rmsErrorInches;
            this.maxErrorInches = maxErrorInches;
            this.sampleCount = sampleCount;
        }

        private static Calibration notConfigured(int sampleCount) {
            return new Calibration(unconfigured(), Double.NaN, Double.NaN, sampleCount);
        }

        public HomographyFloorProjection getProjection() {
            return projection;
        }

        public boolean isConfigured() {
            return projection.isConfigured();
        }

        public double getRmsErrorInches() {
            return rmsErrorInches;
        }

        public double getMaxErrorInches() {
            return maxErrorInches;
        }

        public int getSampleCount() {
            return sampleCount;
        }
    }

    private static final class Similarity {
        private final double scale;
        private final double centerX;
        private final double centerY;

        private Similarity(double scale, double centerX, double centerY) {
            this.scale = scale;
            this.centerX = centerX;
            this.centerY = centerY;
        }

        private double applyX(double x) {
            return scale * (x - centerX);
        }

        private double applyY(double y) {
            return scale * (y - centerY);
        }

        private double[] matrix() {
            return new double[] {
                    scale, 0.0, -scale * centerX,
                    0.0, scale, -scale * centerY,
                    0.0, 0.0, 1.0
            };
        }

        private double[] inverseMatrix() {
            double inverseScale = 1.0 / scale;
            return new double[] {
                    inverseScale, 0.0, centerX,
                    0.0, inverseScale, centerY,
                    0.0, 0.0, 1.0
            };
        }
    }
}
