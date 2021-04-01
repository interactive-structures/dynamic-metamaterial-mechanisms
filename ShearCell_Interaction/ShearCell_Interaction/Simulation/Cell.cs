using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;

namespace ShearCell_Interaction.Simulation
{
    public abstract class Cell
    {
        private List<Vertex> _cellVertices;
        private bool _areVerticesInitialized = false;
 
        protected double _edgeLengthCCW;
        protected double _edgeLengthCW;
        private Cell other;

        protected delegate double OperatorDelegate(double x, double y);

        public List<int> AlreadyDeformedVertexIndices { get; set; } 
        public bool IsCollision { get; protected set; }

        public Vertex IndexVertex { get; set; }

        public List<Edge> CellEdges { get; set; }

        public List<Vertex> CellVertices {
            get
            {
                if (_cellVertices.Count >= 4 && !_areVerticesInitialized)
                {
                    _edgeLengthCCW = (_cellVertices[1].ToVector() - IndexVertex.ToVector()).Length;
                    _edgeLengthCW = (_cellVertices[3].ToVector() - IndexVertex.ToVector()).Length;
                    _areVerticesInitialized = true;
                }
                return _cellVertices;
            }
            set
            {
                _cellVertices = value;
            }
        }

        public List<HashSet<Edge>> CellConstraints { get; protected set; } 

        protected Cell()
        {
            CellVertices = new List<Vertex>();
            CellEdges = new List<Edge>();
            CellConstraints = new List<HashSet<Edge>>();
            AlreadyDeformedVertexIndices = new List<int>();
        }

        public abstract void SetInitialDeformedVertices(Vertex movedVertex, Vector targetOffset);
        public abstract void Deform();
        public abstract bool CanMove();
        public abstract void SetConstraints();

        public abstract double GetMaxDiagonalLength();
        public abstract string GetEdgeStyle(bool isDeformed);
        public abstract string GetVertexStyle(bool isDeformed);


        protected void TranslateCell()
        {
            var fixedIndex = AlreadyDeformedVertexIndices[0];
            var fixedVertex = CellVertices[fixedIndex];
            var offset = Vector.Subtract(fixedVertex.ToVector(), fixedVertex.ToInitialVector());
            //Debug.WriteLine("offset {{{0}, {1}}}", offset.X, offset.Y);

            CellVertices[(fixedIndex + 1) % 4].SetPosition(Vector.Add(CellVertices[(fixedIndex + 1) % 4].ToInitialVector(), offset));
            CellVertices[(fixedIndex + 2) % 4].SetPosition(Vector.Add(CellVertices[(fixedIndex + 2) % 4].ToInitialVector(), offset));
            CellVertices[(fixedIndex + 3) % 4].SetPosition(Vector.Add(CellVertices[(fixedIndex + 3) % 4].ToInitialVector(), offset));
        }

        protected void RotateCell()
        {
            var angle = GetAngleBetweenVertices(CellVertices[AlreadyDeformedVertexIndices[0]], CellVertices[AlreadyDeformedVertexIndices[1]]);
            //Debug.WriteLine("angle of deformed vertices: " + angle * MathHelper.RadToDeg);

            var fixedIndex = AlreadyDeformedVertexIndices[0];
            var diagonalIndex = MathHelper.Mod(AlreadyDeformedVertexIndices[0] + 2, 4);
            var diagonalLength = Math.Sqrt(_edgeLengthCCW * _edgeLengthCCW + _edgeLengthCW * _edgeLengthCW);

            var undeformedDiagonal = Vector.Subtract(CellVertices[diagonalIndex].ToInitialVector(), CellVertices[fixedIndex].ToInitialVector());
            var diagonal = MathHelper.AddVectorAngle(undeformedDiagonal, angle);

            //Debug.WriteLine("undeformed diagonal {{{0}, {1}}} --- deformed diagonal {{{2}, {3}}}", undeformedDiagonal.X, undeformedDiagonal.Y, diagonal.X, diagonal.Y);

            diagonal = MathHelper.SetVectorLength(diagonal, diagonalLength);

            CalculateVerticesFromDiagonal(CellVertices[fixedIndex], diagonal);
        }

        protected void CalculateVerticesFromDiagonal(Vertex anchoredVertex, Vector diagonal)
        {
            var anchoredVertexIndex = CellVertices.IndexOf(anchoredVertex);
            var edgeLengthCCW = anchoredVertexIndex % 2 == 1 ? _edgeLengthCW : _edgeLengthCCW;
            var edgeLengthCW = anchoredVertexIndex % 2 == 0 ? _edgeLengthCW : _edgeLengthCCW;

            OperatorDelegate add = (x, y) => x + y;
            OperatorDelegate subtract = (x, y) => x - y;

            try
            {
                var vectorCCW = CalculateVertex(diagonal, edgeLengthCCW, edgeLengthCW, subtract);
                var vectorCW = CalculateVertex(diagonal, edgeLengthCW, edgeLengthCCW, add);

                CellVertices[(anchoredVertexIndex + 1) % 4].SetPosition(Vector.Add(anchoredVertex.ToVector(), vectorCCW));
                CellVertices[(anchoredVertexIndex + 2) % 4].SetPosition(Vector.Add(anchoredVertex.ToVector(), diagonal));
                CellVertices[(anchoredVertexIndex + 3) % 4].SetPosition(Vector.Add(anchoredVertex.ToVector(), vectorCW));
            }
            catch (Exception exception)
            {
                Debug.WriteLine(exception.Message);
            }
        }

        protected static Vector CalculateVertex(Vector diagonal, double adjacentLeg, double oppositeLeg, OperatorDelegate calcOperator)
        {
            var angleBetweenDiagonalAndLeg = Math.Acos((Math.Pow(adjacentLeg, 2) + Math.Pow(diagonal.Length, 2) - Math.Pow(oppositeLeg, 2))
                                                        / (2 * adjacentLeg * diagonal.Length));
            //var angleDeg = angleDiagonal * 180 / Math.PI; //for debug

            if (Double.IsNaN(angleBetweenDiagonalAndLeg))
                throw new Exception("diagonal angle is NaN, check math!");

            var horizontalVector = new Vector(1, 0);
            var diagonalAngle = Vector.AngleBetween(horizontalVector, diagonal) * Math.PI / 180;
            var angle = calcOperator(diagonalAngle, angleBetweenDiagonalAndLeg);

            var localVector = new Vector(adjacentLeg * Math.Cos(angle), adjacentLeg * Math.Sin(angle));

            if (Math.Abs(localVector.Length - adjacentLeg) > 0.00001)
                throw new Exception("edge lengths changed, check math!");

            return localVector;
        }

        protected double GetAngleBetweenVertices(Vertex baseVertex, Vertex movedVertex)
        {
            var undeformed = Vector.Subtract(movedVertex.ToInitialVector(), baseVertex.ToInitialVector());
            var deformed = Vector.Subtract(movedVertex.ToVector(), baseVertex.ToVector());
            var angle = Vector.AngleBetween(undeformed, deformed) * MathHelper.DegToRad;

            return angle;
        }

        protected double GetEdgeLengthBetweenVertices(int vertexIndex1, int vertexIndex2)
        {
            if (vertexIndex1 % 2 == 0)
                return (vertexIndex1 + 1) % 4 == vertexIndex2 ? _edgeLengthCCW : _edgeLengthCW;

             return (vertexIndex1 + 1) % 4 == vertexIndex2 ? _edgeLengthCW : _edgeLengthCCW;
        }

        protected double GetDiagonalLenth()
        {
            return Math.Sqrt(_edgeLengthCCW * _edgeLengthCCW + _edgeLengthCW * _edgeLengthCW);
        }

        protected void CheckEdgesForCoherence()
        {
            for (var i = 0; i < CellVertices.Count; i++)
            {
                var currentLength =
                    Vector.Subtract(CellVertices[i].ToVector(), CellVertices[(i + 1)%4].ToVector()).Length;
                var targetLength = GetEdgeLengthBetweenVertices(i, (i + 1)%4);

                var difference = currentLength - targetLength;
                if (Math.Abs(difference) > MathHelper.EPSILON)
                    Debug.WriteLine("         wrong edge length! diff={0}, between vertices {1} and {2} at {3}",
                        difference, i, i + 1, this);
            }
        }
    }
}
