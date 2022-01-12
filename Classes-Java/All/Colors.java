package frc.robot;

public class Colors {

	public int C=-1, R=-1, G=-1, B=-1;
	public boolean isGoodReading = false;
	
	public Colors(){
	}

	protected Colors(Colors another)
	{
		this.copyFrom(another);
	}

	protected void copyFrom(Colors another) // similar to Apache commons
	{
		C = another.C;
		R = another.R;
		G = another.G;
		B = another.B;
		isGoodReading = another.isGoodReading;
	}

	public String toString() {
		return String.format("isGood=%b, C=%d, R=%d, G=%d, B=%d", isGoodReading, C, R, G, B);
	}

	public Colors clone()
	{
		return new Colors(this);
	}
}