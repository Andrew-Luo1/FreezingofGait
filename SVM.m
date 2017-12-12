function SVM( X, y, C, max_iter)
%Graph the SVM, provide coefficients for the decision boundary. Max_iter
%can be around 20. X is a two column array containing feature values. y is
%a vector storing the class information. (0 or 1; freezing or normal)


    %Normalize data. Uncomment if magnitude of features too different 
%     sd = std(X)
%     meanVal = mean(X)
%     X(:, 1) = (X(:, 1) - meanVal(1))./sd(1);
%     X(:, 2) = (X(:, 2) - meanVal(2))./sd(2);
    
  
    model = svmTrain(X, y, C, @linearKernel, 1e-3, max_iter);
    w = model.w;
    b = model.b;
    xp = linspace(min(X(:,1)), max(X(:,1)), 100);
    yp = - (w(1)*xp + b)/w(2);
    plotData(X, y);

    hold on;
    plot(xp, yp, '-b'); 
    hold on;
    xlabel('Max Angle (degree)');
    ylabel('Gait Frequency (Hz)');
    
    hold off
    model.w
    model.b
    

end
