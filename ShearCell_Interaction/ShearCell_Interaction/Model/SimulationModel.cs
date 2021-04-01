using System;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Model
{
    public class SimulationModel
    {
        public MetamaterialModel Model { get; set; }
        public ViewModel ViewModel { get; set; }

        public int MaxNumIterations;

        public int MinTargetDof = 4;
        public int MaxTargetDof = 6;
        public static double MinErrorDelta = .001;
        public static double MinImprovement = .01;

        //http://www.wolframalpha.com/input/?i=1%2F(1+%2B+exp(.5+%2F+(50*+0.95%5Ek)));+k+from+0+to+250;

        public double SimAnnStartTemperature;

        //0.99 is very slow falloff. 0.9 is steep falloff after approx. 80 iterations
        public static double SimAnnFalloff = 0.92;

        public bool EnableSimulatedAnnealing;

        public int CurrentNumIterations;

        public double StartError;
        public double CurrentError;
        public int CurrentDof;
        public double MinError;

        public bool StartErrorWasSet;
        public int NumAcceptedSolutions;
        public int NumAcceptedSolutionsSimAnn;
        public int NumRejectedSolutions;

        public double InputWeight = 0.2;
        public double OutputWeight = 0.8;

        public double InputPathLength = -1;
        public double OutputPathLength = -1;

        public SimulationModel(MetamaterialModel model, ViewModel viewModel)
        {
            Model = new MetamaterialModel(model);
            Model.UpdateConstraintsGraph();

            ViewModel = viewModel;

            MaxNumIterations = ViewModel.NumberIterationsGeneration;

            SimAnnStartTemperature = MaxNumIterations / 3.0;
            EnableSimulatedAnnealing = ViewModel.UseSimulatedAnnealing;

            StartError = Double.PositiveInfinity;
            MinError = Double.PositiveInfinity;
        }

        public void AssignValues(SimulationModel target)
        {
            CurrentDof = target.CurrentDof;

            CurrentError = target.CurrentError;
            MinError = target.MinError;

            StartError = target.MinError;
            StartErrorWasSet = target.StartErrorWasSet;
        }
    }
}