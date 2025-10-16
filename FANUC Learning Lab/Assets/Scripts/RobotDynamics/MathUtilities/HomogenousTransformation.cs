using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RobotDynamics.MathUtilities
{
    public class HomogenousTransformation : Matrix
    {
        public HomogenousTransformation() : base(new double[4, 4])
        {

        }

        public HomogenousTransformation(RotationMatrix R, Vector offset) : base(new double[4, 4])
        {
            double[,] m = new double[4, 4];
            for (int x = 0; x < 3; x++)
            {
                for (int y = 0; y < 3; y++)
                {
                    m[x, y] = R.matrix[x, y];
                }
            }
            for (int row = 0; row < 3; row++)
            {
                m[row, 3] = offset[row];
            }
            m[3, 3] = 1;

            matrix = m;
        }

        public HomogenousTransformation(Matrix m) : base(m.matrix)
        {
            if (m.NrCols != 4 || m.NrRows != 4)
            {
                throw new Exception("Matrix dimensions must be 4x4 for HT definitions");
            }
        }

        public Vector GetPosition()
        {
            return new Vector(matrix[0, 3], matrix[1, 3], matrix[2, 3]);
        }
        public RotationMatrix GetRotation()
        {
            double[,] m = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int q = 0; q < 3; q++)
                {
                    m[i, q] = matrix[i, q];
                }
            }
            return new RotationMatrix(m);
        }

        /*
        public HomogenousTransformation Inverse()
        {
            // Extraer rotación y traslación
            RotationMatrix R = GetRotation();
            Vector t = GetPosition();

            // Calcular la transpuesta de la rotación (R^T)
            double[,] Rt = new double[3, 3];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Rt[i, j] = R.matrix[j, i]; // Transpuesta
                }
            }

            // Calcular -R^T * t
            double[] newTranslation = new double[3];
            for (int i = 0; i < 3; i++)
            {
                double sum = 0;
                for (int j = 0; j < 3; j++)
                {
                    sum -= Rt[i, j] * t[j]; // -R^T * t
                }
                newTranslation[i] = sum;
            }

            // Construir la matriz inversa 4x4
            double[,] inverseMatrix = new double[4, 4];
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    inverseMatrix[i, j] = Rt[i, j]; // R^T
                }
                inverseMatrix[i, 3] = newTranslation[i]; // -R^T * t
            }
            inverseMatrix[3, 3] = 1; // Última fila [0 0 0 1]

            return new HomogenousTransformation(new Matrix(inverseMatrix));
        }*/
    }
}
