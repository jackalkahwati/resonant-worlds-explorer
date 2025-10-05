import { useMode } from "@/contexts/ModeContext";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { Info } from "lucide-react";

export const WelcomeMessage = () => {
  const { mode } = useMode();

  if (mode !== "explorer") {
    return null;
  }

  return (
    <Alert className="mb-6">
      <Info className="h-4 w-4" />
      <AlertTitle>Welcome to Explorer Mode!</AlertTitle>
      <AlertDescription>
        You&apos;re in Explorer Mode - a simplified interface designed for discovering exoplanets
        and analyzing biosignatures. We&apos;ve focused on the key features to help you get
        started. Switch to Researcher Mode in the top right if you need access to advanced
        features and detailed analysis tools.
      </AlertDescription>
    </Alert>
  );
};