package frc.lib.team254.geometry;

import frc.lib.team254.util.Interpolable;
import frc.lib.team254.util.CSVWritable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
