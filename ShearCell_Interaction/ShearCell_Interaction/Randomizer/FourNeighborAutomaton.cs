using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Simulation;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Randomizer
{
    public enum CellType
    {
        SHEAR,
        RIGID,
        VOID
    }

    class FourNeighborAutomaton : Automaton
    {
        public FourNeighborAutomaton(MetamaterialModel model, ViewModel viewModel) : base(model, viewModel)
        { }

        //public FourNeighborAutomaton(Model model, ViewModel viewModel)
        //{
        //    _model = model;
        //    _viewModel = viewModel;

        //    Grid = new Dictionary<Vector, CellType>(_gridSize*_gridSize);
        //    ConstructInitialGrid();

        //    CurrentIteration = 0;
        //    HasNextIteration = true;
        //}


        protected override int GetNeighborSum(Vector cellPosition)
        {
            var sum = -1;

            var x = new Vector(cellPosition.X + 1, cellPosition.Y);
            if (Grid.ContainsKey(x))
            {
                if (!_isInitiallyRigid)
                    sum += Grid[x] == CellType.RIGID ? 1 : 0;
                else
                    sum += Grid[x] == CellType.RIGID ? 0 : 1;
            }

            var y = new Vector(cellPosition.X, cellPosition.Y + 1);
            if (Grid.ContainsKey(y))
            {
                if (!_isInitiallyRigid)
                    sum += Grid[y] == CellType.RIGID ? 1 : 0;
                else
                    sum += Grid[y] == CellType.RIGID ? 0 : 1;
            }

            var xy = new Vector(cellPosition.X + 1, cellPosition.Y + 1);
            if (Grid.ContainsKey(xy))
            {
                if (!_isInitiallyRigid)
                    sum += Grid[xy] == CellType.RIGID ? 1 : 0;
                else
                    sum += Grid[xy] == CellType.RIGID ? 0 : 1;
            }

            return sum;
        }

        protected override CellType EvaluateRules(int neighborSum)
        {
            if (neighborSum == 1)
                return CellType.RIGID;
            if (neighborSum == 2)
                return CellType.SHEAR;

            return GetRandomType(new[] { CellType.SHEAR, CellType.RIGID });
        }

    }
}
