using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DijkstraWPF
{
    public class Point2D
    {
        public double X { get; set; }
        public double Y { get; set; }

        public Point2D(double x, double y)
        {
            X = x;
            Y = y;
        }

        public double DistanceTo(Point2D other)
        {
            double dx = X - other.X;
            double dy = Y - other.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        public override bool Equals(object obj)
        {
            return obj is Point2D d &&
                   Math.Abs(X - d.X) < 0.0001 &&
                   Math.Abs(Y - d.Y) < 0.0001;
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(X, Y);
        }
    }

}
