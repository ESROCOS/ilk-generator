package eu.esrocos.kul.robot.generator.common;

import eu.esrocos.kul.robot.kinDsl.FloatLiteral;
import eu.esrocos.kul.robot.kinDsl.Vector3;


public class NumericUtils
{
    public static class IProperties
    {
        public double mass = 0.0;
        public Vector3D com = new Vector3D(0.0, 0.0, 0.0);
        public double ixx = 0.0;
        public double ixy = 0.0;
        public double ixz = 0.0;
        public double iyy = 0.0;
        public double iyz = 0.0;
        public double izz = 0.0;

        public IProperties() {}
        public IProperties(IProperties rhs) {
            mass  = rhs.mass ;
            com.x = rhs.com.x;
            com.y = rhs.com.y;
            com.z = rhs.com.z;
            ixx   = rhs.ixx  ;
            iyy   = rhs.iyy  ;
            izz   = rhs.izz  ;
            ixy   = rhs.ixy  ;
            ixz   = rhs.ixz  ;
            iyz   = rhs.iyz  ;
        }
    }

    /**
     * Computes the inertia properties of a rigid body in a different frame.
     *
     * The frame in which the properties (CoM position and inertia tensor)
     * are currently expressed is C; the new frame in which such parameters
     * have to be expressed is N (C=Current, N=new).
     * The arguments of this function specify the transformation between C and N.
     * If the last argument 'inverse' is false, then the rotation/translation
     * parameters encode the pose of N with respect to C. Otherwise they represent
     * the pose of C with respect to N.
     *
     * The translation is expressed in the current frame. Rotation values are
     * basically Euler angles, consecutive rotations about x, y, and z axis, in
     * this order, of the moving frame: each rotation is about the axis rotated
     * by the previous one ("intrinsic" rotations).
     *
     * This method does NOT support parametric properties, ie it only works
     * when the given inertia properties are all floating point constants.
     *
     * @param inertia the input properties to be expressed in the new frame
     * @param tx translation along the X axis
     * @param ty translation along the Y axis
     * @param tz translation along the Z axis
     * @param rx rotation about the X axis
     * @param ry rotation about the Y axis
     * @param rz rotation about the Z axis
     * @param inverse if false, the previous arguments tell the pose of N wrt C;
     *        if true they express the pose of C wrt N
     * @return a new instance of IProperties that encode the same inertia as the
     *         input, but with coordinates of the new frame N
     */
    public static IProperties rototranslate(IProperties in,
            double tx, double ty, double tz, double rx, double ry, double rz, boolean inverse)
    {
        IProperties out= new IProperties( in );

        double[][] M = rotated_X_original(rx, ry, rz);
        double tmp = 0;
        if(inverse) {
            // The numbers encode the roto-translation to move from N to C, so I need
            //  to invert them in order to have the transformation C --> N, to express
            //  in N the inertia-parameters currently expressed in C
            // Rotate and invert the translation vector
            double tmpx, tmpy, tmpz;
            tmpx = M[0][0] * tx + M[0][1] * ty + M[0][2] * tz;
            tmpy = M[1][0] * tx + M[1][1] * ty + M[1][2] * tz;
            tmpz = M[2][0] * tx + M[2][1] * ty + M[2][2] * tz;
            tx = -tmpx;
            ty = -tmpy;
            tz = -tmpz;
            //transpose M
            tmp = M[0][1];
            M[0][1] = M[1][0];
            M[1][0] = tmp;

            tmp = M[0][2];
            M[0][2] = M[2][0];
            M[2][0] = tmp;

            tmp = M[1][2];
            M[1][2] = M[2][1];
            M[2][1] = tmp;
        }

        // Compute the COM position in the new frame, applying the translation
        //  and the rotation matrix defined by rx,ry,rz ...
        // Translation:
        //   Position vector of the COM, with respect to the frame with origin as in N
        //   but same orientation as C:
        final double comx_N = in.com.x - tx;
        final double comy_N = in.com.y - ty;
        final double comz_N = in.com.z - tz;
        // Rotation:
        out.com.x = M[0][0]*comx_N + M[0][1]*comy_N + M[0][2]*comz_N;
        out.com.y = M[1][0]*comx_N + M[1][1]*comy_N + M[1][2]*comz_N;
        out.com.z = M[2][0]*comx_N + M[2][1]*comy_N + M[2][2]*comz_N;

        // Now computes the inertia tensor in the new frame

        // Parallel axis theorem; go from C to COM, and from COM to N, with two subsequent translations
        out.ixx += in.mass * (comy_N*comy_N + comz_N*comz_N - in.com.y*in.com.y - in.com.z*in.com.z);
        out.iyy += in.mass * (comx_N*comx_N + comz_N*comz_N - in.com.x*in.com.x - in.com.z*in.com.z);
        out.izz += in.mass * (comx_N*comx_N + comy_N*comy_N - in.com.x*in.com.x - in.com.y*in.com.y);
        out.ixy += in.mass * (comx_N*comy_N - in.com.x*in.com.y);
        out.ixz += in.mass * (comx_N*comz_N - in.com.x*in.com.z);
        out.iyz += in.mass * (comy_N*comz_N - in.com.y*in.com.z);

        // Now consider the rotation of the axes.
        // The equations to transform the inertia-moments are very similar to the
        // equation in matrix form for the inertia tensor
        //  I' =  M * I * M^T
        // but some signs are different! Remember that our convention
        // states that ixy, ixz and iyz are the centrifugal moments, and NOT the elements of
        // the inertia tensor; they differ only for the minus sign.
        double tmp_ixx, tmp_iyy, tmp_izz, tmp_ixy, tmp_ixz, tmp_iyz;
        // Ixx
        tmp_ixx = M[0][0] * M[0][0] * out.ixx +
                  M[0][1] * M[0][1] * out.iyy +
                  M[0][2] * M[0][2] * out.izz +
             -2 * M[0][0] * M[0][1] * out.ixy +
             -2 * M[0][0] * M[0][2] * out.ixz +
             -2 * M[0][1] * M[0][2] * out.iyz;
        // Iyy
        tmp_iyy = M[1][0] * M[1][0] * out.ixx +
                  M[1][1] * M[1][1] * out.iyy +
                  M[1][2] * M[1][2] * out.izz +
             -2 * M[1][0] * M[1][1] * out.ixy +
             -2 * M[1][0] * M[1][2] * out.ixz +
             -2 * M[1][1] * M[1][2] * out.iyz;
        // Izz
        tmp_izz = M[2][0] * M[2][0] * out.ixx +
                  M[2][1] * M[2][1] * out.iyy +
                  M[2][2] * M[2][2] * out.izz +
             -2 * M[2][0] * M[2][1] * out.ixy +
             -2 * M[2][0] * M[2][2] * out.ixz +
             -2 * M[2][1] * M[2][2] * out.iyz;
        // Ixy
        tmp_ixy =- M[0][0] * M[1][0] * out.ixx +
                 - M[0][1] * M[1][1] * out.iyy +
                 - M[0][2] * M[1][2] * out.izz +
             (M[0][0]*M[1][1] + M[0][1]*M[1][0]) * out.ixy +
             (M[0][0]*M[1][2] + M[0][2]*M[1][0]) * out.ixz +
             (M[0][1]*M[1][2] + M[0][2]*M[1][1]) * out.iyz;
        // Ixz
        tmp_ixz =- M[0][0] * M[2][0] * out.ixx +
                 - M[0][1] * M[2][1] * out.iyy +
                 - M[0][2] * M[2][2] * out.izz +
            (M[0][0]*M[2][1] + M[0][1]*M[2][0]) * out.ixy +
            (M[0][0]*M[2][2] + M[0][2]*M[2][0]) * out.ixz +
            (M[0][1]*M[2][2] + M[0][2]*M[2][1]) * out.iyz;
        // Iyz
        tmp_iyz =- M[1][0] * M[2][0] * out.ixx +
                 - M[1][1] * M[2][1] * out.iyy +
                 - M[1][2] * M[2][2] * out.izz +
             (M[1][0]*M[2][1] + M[1][1]*M[2][0]) * out.ixy +
             (M[1][0]*M[2][2] + M[1][2]*M[2][0]) * out.ixz +
             (M[1][1]*M[2][2] + M[1][2]*M[2][1]) * out.iyz;

        out.ixx = tmp_ixx;
        out.iyy = tmp_iyy;
        out.izz = tmp_izz;
        out.ixy = tmp_ixy;
        out.ixz = tmp_ixz;
        out.iyz = tmp_iyz;
        return out;
    }

    /**
     * Rounds each element of the given matrix according to the factor.
     *
     * Basically:
     *     mx[.][.] = round(mx[.][.] * factor) / factor
     *
     * @param mx
     * @param factor
     */
    public static void round(double[][] mx, int factor)
    {
        for(int r=0; r<mx.length; r++) {
            for(int c=0; c<mx[0].length; c++) {
                mx[r][c] = Math.round(mx[r][c]*factor)/ (double)(factor);
            }
        }
    }

	public static float invert(float num) {
	    return -num;
	}
	public static boolean isZero(float num) {
		return num==0.0;
	}
	public static float mult(float a, float b) {
	    return a*b;
	}
	public static float div(float a, float b) {
	    return a/b;
	}

	public static double length(Vector3 vec) {
	    if( ! (vec.getX() instanceof FloatLiteral) ||
            ! (vec.getY() instanceof FloatLiteral) ||
            ! (vec.getZ() instanceof FloatLiteral))
        {
            throw new RuntimeException("Cannot compute the length of a vector defined with some variables");
        }
        float x = ((FloatLiteral)vec.getX()).getValue();
        float y = ((FloatLiteral)vec.getY()).getValue();
        float z = ((FloatLiteral)vec.getZ()).getValue();
        return Math.sqrt(x*x + y*y + z*z);
    }

    /**
     * Creates and returns a 3x3 rotation matrix corresponding to the rotation angles
     * in the arguments. The rotation matrix is in the form rotated_X_original, that
     * is, it multiplies coordinate vectors expressed in a reference frame and gives
     * back the coordinates in the frame which is rotated with respect to the first one.
     * The arguments are interpreted as consecutive rotations about the x, y and z
     * axis, all expressed in radians.
     * @param rx the rotation amount about the x axis
     * @param ry the rotation amount about the y axis as it results after the first rotation
     * @param rz the rotation amount about the z axis, after the two previous rotations
     * @return a 3x3 rotation matrix that transforms coordinate vectors according to
     *          the rotation specified in the arguments
     */
    public static double[][] rotated_X_original(double rx, double ry, double rz) {
        double sx = Math.sin(rx);
        double sy = Math.sin(ry);
        double sz = Math.sin(rz);
        double cx = Math.cos(rx);
        double cy = Math.cos(ry);
        double cz = Math.cos(rz);

        // The following is the matrix M that transform coordinates in the original frame
        //  into coordinates of the new frame:  new_X_current
        // [  cos(ry)*cos(rz)    cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz)     sin(rx)*sin(rz) - cos(rx)*sin(ry)*cos(rz) ]
        // [                                                                                                             ]
        // [ - cos(ry)*sin(rz)   cos(rx)*cos(rz) - sin(rx)*sin(ry)*sin(rz)     cos(rx)*sin(ry)*sin(rz) + sin(rx)*cos(rz) ]
        // [                                                                                                             ]
        // [      sin(ry)                    - sin(rx)*cos(ry)                              cos(rx)*cos(ry)              ]
        double[][] M = {
                { cy*cz,   cx*sz + sx*sy*cz,     sx*sz - cx*sy*cz },
                {-cy*sz,   cx*cz - sx*sy*sz,     cx*sy*sz + sx*cz },
                {  sy  ,     - sx*cy       ,     cx*cy }
                };
        return M;
	}

	/**
	 * Computes the 3 rotation parameters (ie Euler angles) given a 3x3 rotation matrix.
	 * This is basically the inverse operation of rotated_X_original(double, double, double)
	 * See the documentation of such a function for the semantics of the angles and the matrix.
	 * @param mx a 3x3 rotation matrix in the form 'rotated_X_original'
	 * @return a vector of three elements, representing the consecutive rotations about the
	 *          x, y, and z axis that correspond to the given matrix
	 */
	public static double[] get_rxryrz(double mx[][]) {
	    double rotx = Math.atan2(-mx[2][1], mx[2][2]); // atan( sx cy , cx cy ) = atan( sx,cx )
	    double sx = Math.sin(rotx);
	    double roty = Math.atan2(mx[2][0], mx[2][1]/(-sx)); // atan( sy / (sx cy / sx) ) = atan( sy/cy )
	    double rotz = Math.atan2(-mx[1][0], mx[0][0]);

	    double[] ret = {rotx, roty, rotz};
	    return ret;
	}
    /**
     * See rotated_X_original()
     */
    public static double[][] original_X_rotated(double rx, double ry, double rz) {
        double sx = Math.sin(rx);
        double sy = Math.sin(ry);
        double sz = Math.sin(rz);
        double cx = Math.cos(rx);
        double cy = Math.cos(ry);
        double cz = Math.cos(rz);

        double[][] M = {
                {     cy*cz      ,       -cy*sz      ,      sy   },
                {cx*sz + sx*sy*cz,   cx*cz - sx*sy*sz,   - sx*cy },
                {sx*sz - cx*sy*cz,   cx*sy*sz + sx*cz,     cx*cy }
                };
        return M;
    }

    /**
     * Multiplies a 3x3 matrix with a 3x1 column vector.
     * @param mx
     * @param v
     * @return
     */
    public static Vector3D matrix3x3Mult(double[][] mx, Vector3D v)
    {
        Vector3D ret = new Vector3D(.0,.0,.0);

        ret.x = mx[0][0] * v.x + mx[0][1] * v.y + mx[0][2] * v.z;
        ret.y = mx[1][0] * v.x + mx[1][1] * v.y + mx[1][2] * v.z;
        ret.z = mx[2][0] * v.x + mx[2][1] * v.y + mx[2][2] * v.z;

        return ret;
    }
    /**
     * Multiplies two 3x3 matrices
     * @param mx1
     * @param mx2
     * @return
     */
    public static double[][] matrix3x3Mult(double[][] mx1, double[][] mx2)
    {
        double[][] ret = new double[3][3];

        double tmp = 0;
        for(int r=0; r<ret.length; r++) {
            for(int c=0; c<ret[0].length; c++) {
                for(int k=0; k<mx2.length; k++) {
                    tmp += mx1[r][k] * mx2[k][c];
                }
                ret[r][c] = tmp;
                tmp = 0;
            }
        }
        return ret;
    }

    public static double[][] transpose3x3(double[][] mx)
    {
        double[][] ret = new double[3][3];

        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                ret[r][c] = mx[c][r];
            }
        }
        return ret;
    }


	/**
	 * Converts the Euler angles representing rotations about moving axes into
	 * the equivalent rotation angles about fixed axis. This function only works
	 * for X-Y-Z rotations.
	 *
	 * The rotation parameters of a reference frame in the Kinematics DSL are
	 * in fact successive X-Y-Z rotation angles. With this function, the
	 * rotation specification can be converted to be used whenever the other
	 * convention is used.
	 *
	 * All values in radians.
	 *
	 * @param rx rotation about the X axis
	 * @param ry rotation about the rotated Y axis
	 * @param rz rotation about the final Z axis
	 * @return an array of three values, with the rotation angles about fixed
	 *         axes that would lead to the same orientation specified in the
	 *         arguments.
	 */
	public static double[] intrinsicToExtrinsic_xyz(double rx, double ry, double rz)
	{
	    double[] ret = new double[3];
	    double sx = Math.sin(rx);
	    double cx = Math.cos(rx);

	    double sy = Math.sin(ry);
	    double cy = Math.cos(ry);

	    double sz = Math.sin(rz);
	    double cz = Math.cos(rz);

	    ret[0] = Math.atan2(cx*sy*sz + sx*cz, cx*cy); // rx2
	    ret[1] = Math.asin(cx*sy*cz - sx*sz); // ry2
	    ret[2] = Math.atan2(cx*sz+sx*sy*cz, cy*cz); // rz2
	    return ret;
	}

	public static void print(double[][] matrix) {
	    for (int i = 0; i < matrix.length; i++) {
	        for (int j = 0; j < matrix[i].length; j++) {
	            System.out.print(matrix[i][j] + " ");
	        }
	        System.out.println();
	    }
	}
}
